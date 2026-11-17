"""唯一读线程负责接收；Session 与全部串口写入由控制线程拥有。"""
from collections import deque
import threading
import time
from receiver import Receiver, ReceiveLimits
from thread_priority import current_thread_priority

class ThreadedReceiver:
    def __init__(self,source,session,clock,capacity=64,priority='unchanged'):
        self.priority=priority;self.priority_report=None
        self.source,self.session,self.clock=source,session,clock
        self.capacity=capacity;self.queue=deque();self.lock=threading.Lock()
        self.pending_bytes=0;self._partial=False;self._busy=False;self._polling=False
        self.source_pending=0;self.active=False;self.active_backlog_seen=0
        self.backlog_pending=False;self.backlog_boundary_at=None
        self.backlog_warnings=0;self.backlog_recoveries=0;self.backlog_waits=0
        self.backlog_events=deque(maxlen=64)
        self.pause_requested=False;self.pause_ack=False;self.policy_at=-float('inf');self.resume_pending=False
        self._poll_active=False;self.policy_switches=0;self.resume_wait_counts={}
        self.strict_limits=ReceiveLimits();self.paused_limits=ReceiveLimits(max_read_s=.1,max_poll_s=.1,partial_timeout_s=.1)
        self.error=None;self._error_delivered=False
        self.stop_event=threading.Event();self.done=threading.Event();self.worker=None
        self.poll_count=0;self.maximum_poll_s=0.;self.queued_count=0;self.drained_count=0
        owner=self
        class Sink:
            def tick(self):pass
            def abort(self):pass
            def receive(self,line):owner.enqueue(line,clock())
        class ObservedSource:
            @property
            def in_waiting(self):
                count=source.in_waiting
                owner.source_pending=count
                if (owner._poll_active if owner._polling else owner.active) and isinstance(count,int) and count>128:
                    owner.active_backlog_seen=max(owner.active_backlog_seen,count)
                    with owner.lock:owner._note_backlog(count, 'serial')
                return count
            def read(self,count):return source.read(count)
        self.slow_windows=deque(maxlen=64);self.slow_window_count=0
        self.raw=Receiver(ObservedSource(),Sink(),clock,slow_window=self._slow_window)
    def request_recovery(self, source="control"):
        with self.lock:self._note_backlog(0, source)

    def _slow_window(self, reason, elapsed):
        # Called by reader before parsing: preserve bytes and FAULT order, but
        # fence all candidates until drain + a later receipt on control thread.
        with self.lock:
            self.slow_window_count+=1
            self.slow_windows.append({'reason':reason,'elapsed_s':elapsed,'at':self.clock()})
            self._note_backlog(0, reason)

    @property
    def partial(self):return self._partial or self._busy or self._polling
    @property
    def in_waiting(self):
        return max(self.pending_bytes,self.source_pending)
    def _note_backlog(self, count, source):
        # Caller holds the queue lock. Keep peaks as evidence, never as live bytes.
        if not self.backlog_pending:
            self.backlog_warnings+=1
            self.backlog_events.append({'warning':self.backlog_warnings,'source':source,
                'bytes':count,'detected_at':self.clock(),'drained_at':None,'recovered_at':None})
        self.backlog_pending=True
        self.backlog_boundary_at=None
    def backlog_ready(self, observation):
        """Drain, then require a later host receipt; never retimestamp queued samples.

        This is only a host queue boundary, not proof of device sampling age.
        The caller retains its original command deadline while waiting.
        """
        if not self.lock.acquire(blocking=False):return False
        try:
            if self.error:return False
            if self.backlog_pending:
                self.backlog_waits+=1
                if self.partial or self.in_waiting:return False
                if self.backlog_boundary_at is None:
                    self.backlog_boundary_at=self.clock()
                    self.backlog_events[-1]['drained_at']=self.backlog_boundary_at
                    return False
                if observation is None or observation.received_at <= self.backlog_boundary_at:
                    return False
                self.backlog_pending=False;self.backlog_recoveries+=1
                self.backlog_events[-1]['recovered_at']=self.clock()
            # An inference made before the drain stays unusable even after recovery.
            return (self.backlog_boundary_at is None or
                    (observation is not None and observation.received_at > self.backlog_boundary_at))
        finally:self.lock.release()
    def request_pause(self):
        self.pause_requested=True;self.resume_pending=False;self.active=False
    def _select_policy(self):
        requested=self.pause_requested
        self._previous_poll_limit_s=self.paused_limits.max_poll_s if self.pause_ack and not requested else None
        if requested != self.pause_ack:
            self.policy_at=self.clock();self.policy_switches+=1
        self.raw.limits=self.paused_limits if requested else self.strict_limits
        self.pause_ack=requested
        self._poll_active=self.active or self.resume_pending
    def _resume_wait(self, reason):
        self.resume_wait_counts[reason]=self.resume_wait_counts.get(reason,0)+1
        return False
    def resume_ready(self, observation):
        # Host queue drain and receipt boundary only, not device sampling proof.
        if self.error:return self._resume_wait('reader_error')
        if not self.backlog_ready(observation):return self._resume_wait('backlog_recovery')
        if observation is None:return self._resume_wait('missing_observation')
        if self.partial or self.in_waiting:return self._resume_wait('receive_not_drained')
        if observation.received_at <= self.policy_at:
            return self._resume_wait('need_post_policy_observation')
        if not self.resume_pending:
            if not self.pause_ack:return self._resume_wait('pause_not_acknowledged')
            self.pause_requested=False;self.resume_pending=True
            return self._resume_wait('strict_requested')
        return not self.pause_ack or self._resume_wait('strict_not_acknowledged')
    def enqueue(self,line,received_at):
        with self.lock:
            if len(self.queue)>=self.capacity:
                self.error=self.error or 'receive_queue_overflow'
                return
            self.queue.append((line,received_at));self.pending_bytes+=len(line)+1;self.queued_count+=1
            if self.pending_bytes>128 and (self._poll_active if self._polling else self.active):
                self._note_backlog(self.pending_bytes, 'frame_queue')
    def start(self):
        if self.worker is not None:raise RuntimeError('reader_already_started')
        self.worker=threading.Thread(target=self._run,name='control-serial-reader',daemon=True)
        self.worker.start()
    def _run(self):
        try:
            with current_thread_priority(self.priority) as priority_report:
                self.priority_report=priority_report
                while not self.stop_event.is_set():
                    begin=self.clock()
                    with self.lock:self._polling=True
                    self._select_policy()
                    try:
                        ok = (self.raw.poll(previous_poll_limit_s=self._previous_poll_limit_s)
                              if self._previous_poll_limit_s is not None else self.raw.poll())
                        if not ok:self.error=self.error or self.raw.error or 'reader_failed'
                    finally:
                        with self.lock:
                            self._partial=bool(self.raw._pending)
                            self._polling=False
                    self.poll_count+=1;self.maximum_poll_s=max(self.maximum_poll_s,self.clock()-begin)
                    time.sleep(.0001)
        except BaseException as exc:
            self.error=self.error or 'reader_exception:'+type(exc).__name__
        finally:self.done.set()
    def poll(self):
        self.session.tick()
        if not self.lock.acquire(blocking=False):
            self._busy=True
            return True
        try:
            rows=list(self.queue);self.queue.clear();self.pending_bytes=0;self._busy=False
            error=self.error
        finally:self.lock.release()
        # Preserve every frame and its order, including invalid STATE and FAULT.
        for line,received_at in rows:
            self.session.receive_at(line,received_at);self.drained_count+=1
        if error and not self._error_delivered:
            self._error_delivered=True;self.session.abort()
        return error is None
    def stop_and_join(self):
        if self.worker is None:return True
        self.stop_event.set()
        try:self.worker.join(timeout=2.)
        except KeyboardInterrupt:pass
        return self.done.is_set()
    def report(self):
        return {'architecture':'single_reader_fifo_control_owned_session','reader_done':self.done.is_set(),
                'queue_capacity_frames':self.capacity,'queued_frames':self.queued_count,'drained_frames':self.drained_count,
                'pending_bytes':self.pending_bytes,'last_source_pending_bytes':self.source_pending,
                'active_source_backlog_peak_bytes':self.active_backlog_seen,'error':self.error,'poll_count':self.poll_count,
                'maximum_reader_poll_s':self.maximum_poll_s,
                'slow_windows':{'count':self.slow_window_count,'events':list(self.slow_windows),'hard_window_s':self.raw.hard_window_s},
                'backlog_recovery':{'warnings':self.backlog_warnings,'recoveries':self.backlog_recoveries,
                    'pending':self.backlog_pending,'wait_checks':self.backlog_waits,
                    'events':list(self.backlog_events),'retained_events':64,
                    'scope':'host drain then later receipt; not device sampling freshness'},
                'pause_policy':{'requested':self.pause_requested,'acknowledged':self.pause_ack,'policy_at':self.policy_at if self.policy_at != -float('inf') else None,'resume_pending':self.resume_pending,'switches':self.policy_switches,'resume_wait_counts':dict(self.resume_wait_counts)},'thread_priority':self.priority_report,
                'scope':'receipt timestamps are host reader parse times, not device sampling; no latest-frame overwrite'}

class ReceiverPort:
    """Only close the port after the sole reader has actually returned."""
    def __init__(self,port):self.port=port;self.reader=None;self.reader_stopped=True
    def __enter__(self):self.port.open();return self
    def __exit__(self,*args):
        self.reader_stopped=self.reader is None or self.reader.stop_and_join()
        if not self.reader_stopped:raise RuntimeError('receiver_stop_unconfirmed')
        self.port.close()
