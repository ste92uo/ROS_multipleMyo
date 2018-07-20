import threading


class WorkerThread(threading.Thread):
	
	def __init__(self, group=None, target=None, name=None, *args, **kwargs):
		super(WorkerThread, self).__init__(group, target, name, args=args, kwargs=kwargs)
		self.__args = args
		self.__kwargs = kwargs
		
	def run(self):
		self._main(*self.__args, **self.__kwargs)
	
	def _main(self, *args, **kwargs):
		pass
