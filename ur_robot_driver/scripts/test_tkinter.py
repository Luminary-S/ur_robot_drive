#!/usr/bin/python3
#-*- coding: utf-8 -*-
##############
#Copyright:  2021 CUHK CURI
#Author: Guangli
#Date: 2022-01-15 15:56:16
#LastEditTime: 2022-01-15 16:38:19
#LastEditors: Guangli
#Description: 
#FilePath: /ur_robot_driver/scripts/test_tkinter.py
##############
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from tkinter import *
import hashlib
import time
import re
LOG_LINE_NUM = 0


class MY_GUI():
    def __init__(self, init_window_name):
        self.init_window_name = init_window_name

    #设置窗口

    def set_init_window(self):
        self.init_window_name.title("rostopic echo to list")  # 窗口名
        # self.init_window_name.resizable(width=True, height=True)
        # width = 300
        # height = 200
        # screen_width = self.init_window_name.winfo_screenwidth()
        # screen_height = self.init_window_name.winfo_screenheight()
        # alignstr = '%dx%d+%d+%d' % (width,height, (screen_width-width)/2, (screen_height-height)/2)
        # self.init_window_name.geometry(alignstr)
        #标签
        self.init_data_label = Label(self.init_window_name, text="input")

        self.log_label = Label(self.init_window_name, text="logger")

        self.result_data_label = Label(
            self.init_window_name, text="output")

        #文本框
        self.init_data_Text = Text(
            self.init_window_name, width=40, height=10)  # 原始数据录入框

        self.result_data_Text = Text(
            self.init_window_name, width=60, height=10)  # 处理结果展示
        self.log_scroll = Scrollbar()
        # text = .Text(win, width=30, height=10)
        # side放到窗体的哪一侧,  fill填充
        # self.log_scroll.pack(side=RIGHT, fill=Y)
        # self.log_data_Text.pack(side=LEFT, fill=Y)
        # 关联
        self.log_data_Text = Text(
            self.init_window_name, width=100, height=20)  # 日志框
        self.log_scroll.config(command=self.log_data_Text.yview)
        self.log_data_Text.config(yscrollcommand=self.log_scroll.set)



        #按钮
        self.topic_trans_btn = Button(
            self.init_window_name, text="force", bg="lightblue", width=5, height=1, command=self.force_topic_trans_list)  # 调用内部方法  加()为直接调用
        self.clear_btn = Button(
            self.init_window_name, text="clear", bg="lightblue", width=5, height=1,command=self.clear)
        self.init_data_label.grid(row=0, column=0)
        self.result_data_label.grid(row=0, column=5)

        self.init_data_Text.grid(row=1, column=0, rowspan=4, columnspan=5)
        self.result_data_Text.grid(row=1, column=5, rowspan=4, columnspan=5)        
        
        self.topic_trans_btn.grid(row=5, column=1)
        self.clear_btn.grid(row=5, column=4)
        self.log_label.grid(row=6, column=0, columnspan=10)
        self.log_data_Text.grid(row=7, column=0, rowspan=2, columnspan=10)
        self.log_scroll.grid(row=7, column=10, rowspan=10)

        self.add_joint_trans()
    
    def add_joint_trans(self):

        #[0.8203511238098145, -0.9432128111468714, -0.0330122152911585, -3.0166876951800745, 4.486150741577148, -1.4653261343585413]
        self.joint_trans_btn = Button(
            self.init_window_name, text="jposition", bg="lightblue", width=5, height=1, command=self.joint_topic_trans_list)
        self.joint_trans_btn.grid(row=5, column=2)

    #功能函数
    def joint_topic_trans_list(self):
        src = self.init_data_Text.get(
            1.0, END).strip().replace("\n| ", "")        
        # print(l)
        if src:
            l = re.split("\s*[;,\s,\[,\]]\s*", src)  # ;或空格及后面接很多空格
            print(l)
            try:
                res = [float("%.5f" % float(i))
                    for i in [l[3], l[2], l[1], l[4], l[5], l[6]]]
                # print(res)
                str_res = res.__str__()
                # print(str_res)
                self.result_data_Text.delete(1.0, END)
                self.result_data_Text.insert(1.0, str_res)
                self.write_log_to_Text("[INFO] joint trans happens:  "+str_res)
            except:
                self.result_data_Text.delete(1.0, END)
                self.result_data_Text.insert(1.0, "joint position trans failed")
        else:
            self.write_log_to_Text("ERROR:joint position trans failed")
    
    def clear(self):
        self.init_data_Text.delete(1.0,END)
        self.result_data_Text.delete(1.0, END)
        # pass
    def force_topic_trans_list(self):
        src = self.init_data_Text.get(
            1.0, END).strip().replace("\n| ", "")
        if src:
            l = re.split("[;,\s]\s*", src)
            try:
                res = [float("%.4f" % float(i))
                    for i in [l[2], l[4], l[6], l[9], l[11], l[13]]]
                # print(res)
                str_res = res.__str__()
                # print(str_res)
                self.result_data_Text.delete(1.0, END)
                self.result_data_Text.insert(1.0, str_res)
                self.write_log_to_Text("[INFO] force trans happens:  "+str_res)
            except:
                self.result_data_Text.delete(1.0, END)
                self.result_data_Text.insert(1.0, "joint position trans failed")
        else:
            self.write_log_to_Text("ERROR:force trans failed")


    def str_trans_to_md5(self):
        src = self.init_data_Text.get(
            1.0, END).strip().replace("\n", "").encode()
        #print("src =",src)
        if src:
            try:
                myMd5 = hashlib.md5()
                myMd5.update(src)
                myMd5_Digest = myMd5.hexdigest()
                #print(myMd5_Digest)
                #输出到界面
                self.result_data_Text.delete(1.0, END)
                self.result_data_Text.insert(1.0, myMd5_Digest)
                self.write_log_to_Text("INFO:str_trans_to_md5 success")
            except:
                self.result_data_Text.delete(1.0, END)
                self.result_data_Text.insert(1.0, "字符串转MD5失败")
        else:
            self.write_log_to_Text("ERROR:str_trans_to_md5 failed")

    #获取当前时间

    def get_current_time(self):
        current_time = time.strftime(
            '%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
        return current_time

    #日志动态打印
    def write_log_to_Text(self, logmsg):
        global LOG_LINE_NUM
        current_time = self.get_current_time()
        logmsg_in = str(current_time) + " " + str(logmsg) + "\n"  # 换行
        if LOG_LINE_NUM <= 7:
            self.log_data_Text.insert(END, logmsg_in)
            LOG_LINE_NUM = LOG_LINE_NUM + 1
        else:
            self.log_data_Text.delete(1.0, 2.0)
            self.log_data_Text.insert(END, logmsg_in)


def gui_start():
    init_window = Tk()  # 实例化出一个父窗口
    ZMJ_PORTAL = MY_GUI(init_window)
    # 设置根窗口默认属性
    ZMJ_PORTAL.set_init_window()

    init_window.mainloop()  # 父窗口进入事件循环，可以理解为保持窗口运行，否则界面不展示


gui_start()
