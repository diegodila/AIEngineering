#!/usr/bin/python
# -*- coding: utf-8 -*-

# Programa simples com camera webcam e opencv
import cv2
import numpy as np
import math

#filtro amarelo

image_lower_amarelo = np.array([30,50,70])
image_upper_amarelo = np.array([50,255,255])


#filtro magenta
image_lower_mag = np.array([140,50,80])
image_upper_mag = np.array([180,255,225])




def image_da_webcam(img):
    """
    ->>> !!!! FECHE A JANELA COM A TECLA ESC !!!! <<<<-
        deve receber a imagem da camera e retornar uma imagems filtrada.
    """
    img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)  
    saida = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)

    
    #mascara 
    mask1 = cv2.inRange(img, image_lower_amarelo, image_upper_amarelo)
    mask2 = cv2.inRange(img, image_lower_mag, image_upper_mag)
    
    mask_res = cv2.bitwise_or(mask1, mask2)
    
    contornos, _ = cv2.findContours(mask_res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    contornos_ord = sorted(contornos, key=cv2.contourArea, reverse= True)
    
    # For para calcular a área e o CM, e imprimir na imagem de acordo com a posição do eixo X do CM,\n",
    # para que cada informação fique do lado do circulo correspondente\n",
    cxV = []
    cyV = []
    for i in contornos_ord[0:2]:
        area = cv2.contourArea(i)
        M = cv2.moments(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cxV.append(cx)
            cyV.append(cy)

            tamanho = 20
            cor = (0,0,0)
            cv2.line(saida,(cx - tamanho,cy),(cx + tamanho,cy),cor,5)
            cv2.line(saida,(cx,cy - tamanho),(cx, cy + tamanho),cor,5)
            fonte = cv2.FONT_HERSHEY_SIMPLEX
            texto = cx, cy, area
            origem = (cx,cy-50)
        
            cv2.putText(saida, str(texto), origem, fonte,1,(0,0,0),2,cv2.LINE_AA)
    
            # Traça a reta
            cor = (0, 0, 0)
            vetorTamanho = len(cxV)
            cv2.line(saida,(cxV[0],cyV[0]), (cxV[vetorTamanho-1], cyV[vetorTamanho-1]),cor,5)
        
            # Calcula e imprime o ângulo da reta\n",
            fonte = cv2.FONT_HERSHEY_SIMPLEX
            cxT = cxV[0]-cxV[vetorTamanho-1]
            cyT = cyV[0]-cyV[vetorTamanho-1]

            angulo = math.atan2(cyV[0]-cyV[vetorTamanho-1],cxV[0]-cxV[vetorTamanho-1])
            texto = str(round(math.degrees(angulo), 2))
            origem = (150,150)
            cv2.putText(saida, texto, origem, fonte,1,(127,0,0),2,cv2.LINE_AA)
    

    return saida

cv2.namedWindow("preview")
vc = cv2.VideoCapture("NAC_VIDEO.mp4")  ##### 0 1
vc.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

while rval:
    frame = cv2.flip(frame,1)
    img = image_da_webcam(frame)

    cv2.imshow("preview", img)
    rval, frame = vc.read()
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break

cv2.destroyWindow("preview")
vc.release()
