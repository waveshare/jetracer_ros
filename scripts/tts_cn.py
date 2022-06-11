#!/usr/bin/python
# -*- coding:utf-8 -*-
#
#   合成小语种需要传输小语种文本、使用小语种发音人vcn、tte=unicode以及修改文本编码方式
#  错误码链接：https://www.xfyun.cn/document/error-code （code返回错误码时必看）
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import sys
import websocket
import datetime
import hashlib
import base64
import hmac
import json
import time
import ssl
from wsgiref.handlers import format_date_time
from datetime import datetime
from time import mktime
import os
from subprocess import Popen, PIPE

if(sys.version[:1] == "3"):   #python3
    from urllib.parse import urlencode
    import _thread as thread

else:                         #python2
    from urllib import urlencode
    import thread
    reload(sys)
    sys.setdefaultencoding('utf-8')

import rospy 
from std_msgs.msg import String

FilePath = os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/data" #../data

STATUS_FIRST_FRAME = 0  # 第一帧的标识
STATUS_CONTINUE_FRAME = 1  # 中间帧标识
STATUS_LAST_FRAME = 2  # 最后一帧的标识


class Ws_Param(object):
    # 初始化
    def __init__(self, APPID, APIKey, APISecret):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret

        # 公共参数(common)
        self.CommonArgs = {"app_id": self.APPID}
        # 业务参数(business)，更多个性化参数可在官网查看
        self.BusinessArgs = {"aue": "lame", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan", "tte": "utf8"}

    def set_text(self, text):
        self.Text = text
        if(sys.version[:1] == "3"):
            self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-8')), "UTF8")}
        else:
            self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-8')))}
        #使用小语种须使用以下方式，此处的unicode指的是 utf16小端的编码方式，即"UTF-16LE"”
        #self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-16')), "UTF8")}
        
    # 生成url
    def create_url(self):
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        # 生成RFC1123格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        # 拼接字符串
        signature_origin = "host: " + "ws-api.xfyun.cn" + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + "/v2/tts " + "HTTP/1.1"
        # 进行hmac-sha256进行加密
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = "api_key=\"%s\", algorithm=\"%s\", headers=\"%s\", signature=\"%s\"" % (
            self.APIKey, "hmac-sha256", "host date request-line", signature_sha)
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')
        # 将请求的鉴权参数组合为字典
        v = {
            "authorization": authorization,
            "date": date,
            "host": "ws-api.xfyun.cn"
        }
        # 拼接鉴权参数，生成url
        url = url + '?' + urlencode(v)
        # print("date: ",date)
        # print("v: ",v)
        # 此处打印出建立连接时候的url,参考本demo的时候可取消上方打印的注释，比对相同参数时生成的url与自己代码生成的url是否一致
        # print('websocket url :', url)
        return url

def on_message(ws, message):
    try:
        message =json.loads(message)
        code = message["code"]
        sid = message["sid"]
        audio = message["data"]["audio"]
        audio = base64.b64decode(audio)
        status = message["data"]["status"]
        #print(message)
        if status == 2:
            #print("ws is closed")
            ws.close()
        if code != 0:
            errMsg = message["message"]
            print("sid:%s call error:%s code is:%s" % (sid, errMsg, code))
        else:

            with open(FilePath + '/demo.mp3', 'ab') as f:
                f.write(audio)
            p=Popen("play -q "+ FilePath + "/demo.mp3", stdout=PIPE, shell=True)
            p.wait()
            os.remove(FilePath + '/demo.mp3')
    except Exception as e:
        print("receive msg,but parse exception:", e)



# 收到websocket错误的处理
def on_error(ws, error):
    print("### error:", error)


# 收到websocket关闭的处理
def on_close(ws):
    print("### closed ###")


# 收到websocket连接建立的处理
def on_open(ws):
    def run(*args):
        d = {"common": wsParam.CommonArgs,
             "business": wsParam.BusinessArgs,
             "data": wsParam.Data,
             }
        d = json.dumps(d)
        #print("------>开始发送文本数据")
        ws.send(d)
        if os.path.exists(FilePath + '/demo.mp3'):
            os.remove(FilePath + '/demo.mp3')

    thread.start_new_thread(run, ())

def callback(data):
    rospy.loginfo('I heard %s', data.data)
    websocket.enableTrace(False)
    wsUrl = wsParam.create_url()
    wsParam.set_text(data.data)
    ws = websocket.WebSocketApp(wsUrl, on_message=on_message, on_error=on_error, on_close=on_close)
    ws.on_open = on_open
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

if __name__ == "__main__":
    rospy.init_node('jetracer_tts_node',log_level=rospy.INFO)
    APPID = rospy.get_param('~APPID','58e61263')
    APISecret = rospy.get_param('~APISecret','dffb2043eeb3624eff11f4df0fd84d7d')
    APIKey = rospy.get_param('~APIKey','36d5946d3501388b423ce66f633385f9')
    
    # 测试时候在此处正确填写相关信息即可运行
    wsParam = Ws_Param(APPID=APPID, APISecret=APISecret, APIKey=APIKey)
                       
    rospy.Subscriber('speak', String, callback, queue_size=1)
    rospy.spin()

