from bottle import route, run

@route('/hello')
def hello():
	return "Hello there..."

run(host='192.168.8.39', port=80, debug=True, reloader=True)
