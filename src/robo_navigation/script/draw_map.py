import cv2
import numpy as np

color_gray=(120,120,120)
color_black=(0,0,0)
color_red=(0,0,255)
font =cv2.FONT_HERSHEY_SIMPLEX


map_empty=np.ones([800,500,3],np.uint8)*255
cv2.rectangle(map_empty, (220,0),(250,80),color_gray, -1)
cv2.rectangle(map_empty, (150,180),(270,210),color_gray, -1)
cv2.rectangle(map_empty, (370,120),(400,200),color_gray, -1)
cv2.rectangle(map_empty, (0,310),(200,340),color_gray, -1)

cv2.rectangle(map_empty, (300,460),(500,490),color_gray, -1)
cv2.rectangle(map_empty, (230,590),(350,620),color_gray, -1)
cv2.rectangle(map_empty, (100,600),(130,680),color_gray, -1)
cv2.rectangle(map_empty, (250,720),(280,800),color_gray, -1)

map_pt=map_empty.copy()
pt_self=[(80,80),
(80,160),
(180,130),
(320,130),
(320,60),
(450,60),
(450,260),
(310,260),
(200,260),
(80,260),
(420,360),
(320,360)]

pt_enemy=[]
for i in range(len(pt_self)):
    x=500-pt_self[i][0]
    y=800-pt_self[i][1]
    print "%d  %d" %(x,pt_self[i][0])
    pt_enemy+=[(x,y)]

pt_list=pt_self+pt_enemy+[(250,400)]
for i in range(len(pt_list)):
    cv2.circle(map_pt,pt_list[i],5,color_black,3)
    cv2.putText(map_pt,'%d'%i,pt_list[i],font,1,(255,0,0))

nei_matrix=np.ones([len(pt_list),len(pt_list)])*100

def add_edge(i,j):
    cv2.line(map_pt,pt_list[i],pt_list[j],color_red,2)
    distance=pow(pow((pt_list[i][0]-pt_list[j][0]),2)+pow((pt_list[i][1]-pt_list[j][1]),2),0.5)
    line_center=((pt_list[i][0]+pt_list[j][0])/2,(pt_list[j][1]+pt_list[j][1])/2)
    cv2.putText(map_pt,'%d'%distance,line_center,font,0.5,color_red)
    nei_matrix[i][j]=1
    nei_matrix[j][i]=1

    if (i != len(pt_list)-1):
        i=len(pt_self)+i
    if (j != len(pt_list)-1):    
        j=len(pt_self)+j
    cv2.line(map_pt,pt_list[i],pt_list[j],color_red,2)
    distance=pow(pow((pt_list[i][0]-pt_list[j][0]),2)+pow((pt_list[i][1]-pt_list[j][1]),2),0.5)
    line_center=((pt_list[i][0]+pt_list[j][0])/2,(pt_list[i][1]+pt_list[j][1])/2)
    cv2.putText(map_pt,'%d'%distance,line_center,font,0.5,color_red)
    nei_matrix[i][j]=1
    nei_matrix[j][i]=1

#add_edge(1,2)
add_edge(0,1)
add_edge(0,2)
add_edge(2,3)
add_edge(3,4)
add_edge(4,5)
add_edge(5,6)
add_edge(6,7)
add_edge(3,7)
add_edge(7,8)
add_edge(8,9)
add_edge(1,9)
add_edge(6,10)
add_edge(10,11)
add_edge(7,11)
add_edge(11,24)
add_edge(7,24)

cv2.imshow("map",map_pt)
cv2.waitKey(0)

point_matrix=np.zeros([len(pt_list),2])
for i in range(len(pt_list)):
    point_matrix[i][0]=pt_list[i][0]
    point_matrix[i][1]=pt_list[i][1]
print point_matrix
fs = cv2.FileStorage("../launch/matrix.xml", cv2.FILE_STORAGE_WRITE)
fs.write('Matrix',nei_matrix)
fs.write('Point',point_matrix)
