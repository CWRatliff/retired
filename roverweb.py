from bottle import route, run, template, request
import time

IP_ADDRESS = '192.168.1.2'

@route('/')
def index():
    cmd = request.GET.get('command', '')
    if cmd == 'f':
        print ("f")
    elif cmd == 'l':
        print("l")
    else:
        print("cmd = "+cmd)
    return template('home.tpl')

try:
    run(host=IP_ADDRESS, port=80)
finally:
    print("finally")
