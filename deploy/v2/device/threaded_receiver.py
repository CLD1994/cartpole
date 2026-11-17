"""唯一读线程负责接收；Session 与全部串口写入由控制线程拥有。"""
from collections import deque
import threading
import time
from receiver import Receiver

class ThreadedReceiver:
    def __init__(self,source,session,clock,capacity=64):
        self.source,self.session,self.clock=source,session,clock
        self.capacity=capacity;self.queue=deque();self.lock=threading.Lock()
        self.pending_bytes=0;self._partial=False;self._busy=False;self._polling=False
        self.source_pending=0;self.active=False;self.active_backlog_seen=0
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
                if owner.active and isinstance(count,int) and count>128:
                    owner.active_backlog_seen=max(owner.active_backlog_seen,count)
                return count
            def read(self,count):return source.read(count)
        self.raw=Receiver(ObservedSource(),Sink(),clock)
    @property
    def partial(self):return self._partial or self._busy or self._polling
    @property
    def in_waiting(self):
        return max(self.pending_bytes,self.source_pending,self.active_backlog_seen)
    def enqueue(self,line,received_at):
        with self.lock:
            if len(self.queue)>=self.capacity:
                self.error=self.error or 'receive_queue_overflow'
                return
            self.queue.append((line,received_at));self.pending_bytes+=len(line)+1;self.queued_count+=1
    def start(self):
        if self.worker is not None:raise RuntimeError('reader_already_started')
        self.worker=threading.Thread(target=self._run,name='control-serial-reader',daemon=True)
        self.worker.start()
    def _run(self):
        try:
            while not self.stop_event.is_set():
                begin=self.clock()
                self._polling=True
                try:
                    if not self.raw.poll():self.error=self.error or self.raw.error or 'reader_failed'
                finally:
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
                'scope':'receipt timestamps are host reader parse times, not device sampling; no latest-frame overwrite'}

class ReceiverPort:
    """Only close the port after the sole reader has actually returned."""
    def __init__(self,port):self.port=port;self.reader=None;self.reader_stopped=True
    def __enter__(self):self.port.open();return self
    def __exit__(self,*args):
        self.reader_stopped=self.reader is None or self.reader.stop_and_join()
        if not self.reader_stopped:raise RuntimeError('receiver_stop_unconfirmed')
        self.port.close()
