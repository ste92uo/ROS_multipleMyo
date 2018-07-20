import os
from select import select
from collections import deque
from threading import RLock


class WaitableEvent(object):
	"""
	Provides an abstract object that can be used to resume select loops with
	indefinite waits from another thread or process. This mimics the standard
	threading.Event interface.
	"""

	def __init__(self, name='Anonymous'):
		self._name = name
		self._read_fd, self._write_fd = os.pipe()

	def wait(self, timeout=None):
		rfds, _, _ = select([self._read_fd], [], [], timeout)
		return self._read_fd in rfds

	@property
	def is_set(self):
		return self.wait(0)

	def clear(self):
		if self.is_set:
			os.read(self._read_fd, 1)

	def set(self):
		if not self.is_set:
			os.write(self._write_fd, '1')

	def fileno(self):
		"""
		Return the FD number of the read side of the pipe, allows this object to
		be used with select.select().
		"""
		return self._read_fd

	def __str__(self):
		return 'WaitableEvent[' + self._name + ']'

	def __repr__(self):
		return self.__str__()

	def __del__(self):
		os.close(self._read_fd)
		os.close(self._write_fd)


class WaitableQueue(object):
	"""
	Provides a queue that can be used with the select primitive for multiple
	queues (or other objects) waiting
	"""
	
	def __init__(self, name='Anonymous'):
		self._name = name
		self._lock = RLock()
		self._read_fd, self._write_fd = os.pipe()
		self._queue = deque()
		
	def wait(self, timeout=None):
		try:
			rfds, _, _ = select([self._read_fd], [], [], timeout)
		except KeyboardInterrupt as e:
			return False
		return self._read_fd in rfds
	
	@property
	def name(self):
		return self._name
	
	@property
	def empty(self):
		with self._lock:
			return not self.wait(0)
	
	def clear(self):
		with self._lock:
			b = os.read(self._read_fd, 1024)
			while len(b) == 1024:
				b = os.read(self._read_fd, 1024)
	
	def pop(self):
		with self._lock:
			if not self.empty:
				os.read(self._read_fd, 1)
				return self._queue.pop()
			return None
	
	def push(self, obj):
		with self._lock:
			self._queue.append(obj)
			os.write(self._write_fd, '1')

	def fileno(self):
		"""
		Return the FD number of the read side of the pipe, allows this object to
		be used with select.select().
		"""
		return self._read_fd
	
	def __str__(self):
		return 'WaitableQueue[' + self._name + ']'

	def __repr__(self):
		return self.__str__()

	def __del__(self):
		os.close(self._read_fd)
		os.close(self._write_fd)
