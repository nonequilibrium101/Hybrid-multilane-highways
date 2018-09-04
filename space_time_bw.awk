{
if(NR==1) print "P1";
if(NR==2) print 1024, 1024;
if(NR==3) print "#";
if(NR>3)
for(i=1; i<=NF; i++){
if($i>=0) {if($i<5) print 1;}
if($i==-1) print 0;
if($i==5) print 0;
}
}
