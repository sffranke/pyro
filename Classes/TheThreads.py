import threading
import logging
'''
logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)
'''
class MyThread(threading.Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None):
        
        super().__init__()
        self.args = args
        self.kwargs = kwargs
        return

    
    def run(self):
        #logging.debug('running with %s', self.args)
       
        
        servopin = self.args[0]
        #servorot = self.args[1]
        angle = self.args[1]
        caller = self.args[2]
        transition = self.args[3]
        
        caller.move(servopin, angle, transition)
        
        return
    



