from flask import Flask
from std_msgs.msg import String, Float64MultiArray

app = Flask(__name__)

@app.route('/') # 루트 경로에서 실행되는 함수 즉 홈페이지의 시작부분에서 사용되는 함수를 의미
def hello_world():
    return 'Hello Flask'

@app.route('/asdf') #라면 /asdf 홈페이지에서 실행되는 함수를 설정할 수 있음
def hello_asdf():
    return 'hello asdf'

if __name__ == '__main__':
    app.run()