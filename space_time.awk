{
if(NR<=3)
print $0;
else
for(i=1; i<=NF; i++){
if($i==5) print 0, 255, 0;
if($i==4) print 0, 204, 0;
if($i==3) print 255, 128, 0;
if($i==2) print 255, 0, 0;
if($i==1) print 204, 0, 0;
if($i==0) print 158, 0, 0;
if($i==-1) print 255, 255, 255;
}
}