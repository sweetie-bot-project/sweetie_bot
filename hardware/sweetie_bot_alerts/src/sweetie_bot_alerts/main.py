#!/usr/bin/env python
import rospy
# этот модуль создастся автомитически из файла Letter.msg
from sweetie_bot_herkulex_msgs.msg import HerkulexState

def letterCallback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard [%s]", msg.temperature)

def listener():
    # В ROS все ноды имеют уникальное имя. Если появится вторая нода с
    # тем же именем, то первая будет аварийно заверщена. Параметр
    # anonymous=True указывает, что при запуске будет выбранно
    # уникадбное имя. Анонимные ноды могут быть запущены одновременно,
    # не мешая друг другу.
    rospy.init_node('princess_celestia', anonymous=True)
    # При помощи функции Subscriber() можно сказать ROS, что вы хотите получать сообщения
    # из определенного топика. Сообщения будут переданы в callback функцию letterCallback.
    rospy.Subscriber("test", HerkulexState, letterCallback)

    # spin() не дает скрипту завершиться, пока нода не будет остановлена
    rospy.spin()

def main():
    listener()

