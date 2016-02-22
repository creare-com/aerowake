import threading
import time

def thread_one():
    while True:        
        print 'Thread 1 Fired'
        time.sleep(.5)

def thread_two():
    while True:        
        print 'Thread 2 Fired'
        time.sleep(2)

t1=threading.Thread(target=thread_one)
t2=threading.Thread(target=thread_two)

t1.start
t2.start









