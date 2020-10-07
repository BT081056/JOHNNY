#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  2 14:12:45 2020

@author: nano
"""
#import numpy as np
import RPi.GPIO as GPIO
import time 
import cv2
import os

#tStart=time.time()

#設定畫面中心點的座標
midpoint=[390,390]
thresh=998
input_pin = 18  # BCM pin 18, BOARD pin 12
LED_G=17         # BCM pin 17, BOARD pin 11
LED_R=27        # BCM pin 27, BOARD pin 13
imgpath='/home/nano/Desktop/Edgar_code/test_photo'



#定義如何移除List內重覆的list (先轉成Turple 再轉回來)
def delectdoublelist(listx): 
    s = set(tuple(l) for l in listx)
    listx=[list(t) for t in s]
    return listx

#定義在相同(midpoint)row上,上下兩點的座標並算出長度

def find_Y_length(ListXY_All,column_point,midpoint):
    same_column_list=[]
    for i in range(len(ListXY_All)):
        if ListXY_All[i][1]==column_point:
            same_column_list.append(ListXY_All[i])
            #same_column_list=delectdoublelist(same_column_list)
    list_up=[]
    list_down=[]
    for i in range(len(same_column_list)):
        if same_column_list[i][0]>=midpoint[0]:
            list_up.append(same_column_list[i][0])
        else:
            list_down.append(same_column_list[i][0])

    list_up_point=(min(list_up),column_point)
  
    list_down_point=(max(list_down),column_point)
  
    a_length=list_up_point[0]-list_down_point[0]
    
    return (a_length)

#定義在相同(midpoint) column上,左右兩點的座標並算出長度
def find_X_length(ListXY_All,row_point,midpoint):
        same_row_list=[]
        for i in range(len(ListXY_All)):
            if ListXY_All[i][0]==row_point:
                same_row_list.append(ListXY_All[i])
                #same_row_list=delectdoublelist(same_row_list)
        list_left=[]
        list_right=[]
        for i in range(len(same_row_list)):
            if same_row_list[i][1]<=midpoint[1]:
                list_left.append(same_row_list[i][1])
            else:
                list_right.append(same_row_list[i][1])

        list_left_point=(row_point,max(list_left))
  
        list_right_point=(row_point,min(list_right))
  
        b_length=list_right_point[1]-list_left_point[1]

        return (b_length)


def measure_a_b():
    # 选择摄像头的编号
    cap = cv2.VideoCapture(0)
    # 添加这句是可以用鼠标拖动弹出的窗体
    cv2.namedWindow('real_img', cv2.WINDOW_NORMAL)

    prev_value = None

    # Pin 腳位設置
    GPIO.setmode(GPIO.BCM)  # 設置為BCM模式（您也可以自行改為BOARD模式）
    GPIO.setup(input_pin, GPIO.IN)  # 設置輸入的腳位為input_pin
    GPIO.setup(LED_R, GPIO.OUT)  # 設置輸入的腳位為input_pin
    GPIO.setup(LED_G, GPIO.OUT)  # 設置輸入的腳位為input_pin
    
    GPIO.output(LED_R,GPIO.LOW)
    GPIO.output(LED_G,GPIO.LOW)    
    
    
    while(cap.isOpened()):
        # 读取摄像头的画面
        ret, frame = cap.read()
        # 顯示畫面
        cv2.imshow('real_img', frame)
        
        
        value = GPIO.input(input_pin)
        try:
            if value != prev_value:
                if value == GPIO.HIGH:
                    value_str = "HIGH"
                    print("take a picture")
                    now=time.strftime("%Y-%m-%d-%H:%M:%S",time.localtime())
                    #儲存圖檔
                    #cv2.imwrite(os.path.join(imgpath,now+'.jpg'), frame)
                    #讀取圖檔
                    #img = cv2.imread(os.path.join(imgpath,now+'.jpg'),0)
                    
                    #只選取中心的一部分
                    #大黃擷取的位置
                    x=230
                    y=110
                    w=780
                    h=780
                    img=frame[y:y+h,x:x+w]
                    img = cv2.GaussianBlur(img, (9, 9), 0)

                    canny = cv2.Canny(img, 50, 100)

                    edged = cv2.dilate(canny, None, iterations=2) 

                    edged = cv2.erode(edged, None, iterations=1)
                    
                    contours, hierarchy1 =cv2.findContours(edged.copy(),  cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
                    #print('邊緣數量',len(contours))
                    ListXY_All = []
                    for i in range(len(contours)):#物件數
                        for j in range (len(contours[i])): #i物件的座標數
                            x,y = contours[i][j][0][0],contours[i][j][0][1]
                            ListXY_All.append([x,y])
                    ListXY_All=delectdoublelist(ListXY_All)
                    #找Y軸(row)與midpoint 一樣的點存到list_1
                    #找x軸(column)與midpoint 一樣的點存到list_2
                    list_1=[]
                    list_2=[]
                    for i in range(len(ListXY_All)):
                        if ListXY_All[i][0]==midpoint[0]:
                            list_1.append(ListXY_All[i])

                        if ListXY_All[i][1]==midpoint[1]:
                            list_2.append(ListXY_All[i])
                    #移除重覆座標
                    #list_1=delectdoublelist(list_1)
                    #list_2=delectdoublelist(list_2)
                    #將list1 的座標 以midpoint分成左右兩邊,只留下column座標
                    list_1_left=[]
                    list_1_right=[]
                    for i in range(len(list_1)):
  
                        if list_1[i][1]<=midpoint[1]:
                            list_1_left.append(list_1[i][1])
                        else:
                            list_1_right.append(list_1[i][1])
                    #將list2 的座標 以midpoint的row分成上下兩邊,只留下row座標
                    list_2_up=[]
                    list_2_down=[]
                    for i in range(len(list_2)):
  
                        if list_2[i][0]>=midpoint[0]:
                            list_2_up.append(list_2[i][0])
                        else:
                            list_2_down.append(list_2[i][0])
                    #取得中間輪廓 中List_1最左邊的column座標
                    list1_left_point=[midpoint[0],max(list_1_left)]
                    #取得中間輪廓 中List_2最上面的column座標
                    list2_up_point=[min(list_2_up),midpoint[1]]
                    #取得中間輪廓 中List_1最右邊的column
                    list1_right_point=[midpoint[0],min(list_1_right)]
                    #取得中間輪廓 中List_2最下面的column
                    list2_down_point=[max(list_2_down),midpoint[1]]
                    
                    
                    all_a_length=[]
                    for i in range(list1_left_point[1]+1,list1_right_point[1],30):
                        #for i in range(450,550):
                        try:
                            length=find_Y_length(ListXY_All,i,midpoint)
                            all_a_length.append(length)
                        except:
                            print('skip')
                    all_b_length=[]
                    for i in range(list2_down_point[0]+1,list2_up_point[0],30):
                        #for i in range(550,650):
                        try:
                            length=find_X_length(ListXY_All,i,midpoint)
                            all_b_length.append(length)
                        except:
                            print('skip')
                    a=max(all_a_length)
                    b=max(all_b_length)
                    c=a+b
                    
                    
                    print('邊緣數量',len(contours))
                    
                    
                    
                    
                    
                    print('a軸長',a)
                    print('b軸長',b)
                    
                    
                    
                    
                    
                    
                    if c>=thresh:
                        judge='NG'
                        GPIO.output(LED_R,GPIO.HIGH)
                        print('a+b='+str(c)+'>='+str(thresh) +'judge NG')
                        cv2.putText(img,judge+'='+str(c)+'    '+'a='+str(a)+'  '+'b='+str(b),(50,100),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,0,255),3,cv2.LINE_AA)
                        cv2.imwrite(os.path.join(imgpath,now+'.jpg'), img)
                    else:
                        judge='OK'
                        GPIO.output(LED_G,GPIO.HIGH)
                        #print('a+b<645 judge ok')
                        print('a+b='+str(c)+'<'+str(thresh)+'judge ok')
                        cv2.putText(img,judge+'='+str(c)+'    '+'a length='+str(a)+'  '+'b length='+str(b),(50,100),cv2.FONT_HERSHEY_SIMPLEX,1.5,(0,255,0),3,cv2.LINE_AA)
                        cv2.imwrite(os.path.join(imgpath,now+'.jpg'), img)
                    #tEnd=time.time()
                    #print('It cost %2f sec' %(tEnd-tStart))
                    #儲存圖檔
                    #cv2.putText(img,judge+str(c)+'  '+'a length='+str(a)+'  '+'b length='+str(b),(200,100),cv2.FONT_HERSHEY_SIMPLEX,2,(255,255,255),1,cv2.LINE_AA)
                    #cv2.imwrite(os.path.join(imgpath,now+'.jpg'), img)
                else:
                    value_str = "LOW"
                    #GPIO.cleanup()
                    GPIO.output(LED_R,GPIO.LOW)
                    GPIO.output(LED_G,GPIO.LOW)
                print("Value read from pin {} : {}".format(input_pin,value_str))
                prev_value = value
                
                time.sleep(2)
        
        
            if cv2.waitKey(1) & 0xFF == ord('q'):# 按下'q'就退出
                break
        except:
            print('pick up omnin seal')
        # 释放画面
    cap.release()
    cv2.destroyAllWindows()
        #GPIO.cleanup()

if __name__ == '__main__':
    measure_a_b()