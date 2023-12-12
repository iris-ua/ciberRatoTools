
BEGIN {
   line=1
   score=0
   pathind = 1
   robDist = 0

   linelength=49
   nlines=21

   exiting=0
}

# to read reference map, beacon list and distance from first file
FNR==NR {
   a[NR]=$0
   if(line<=nlines) {
      mapref[nlines+1-line][0] = ""
      nmapref=split(a[line],mapref[nlines+1-line],"") 
   }
   if(line==nlines+1) {
      for (t=1; t<=NF; t++) {
         visited[$t] = 0
      }
   }
   else bestDist = $1
   line++

   next
}

{
    x=$1
    y=$2
    print "x",x,"y",y

    if (pathind==1) {
        if(x!=0 || y!=0) {
             print "Path does not start at 0 0"
             print "Planning score: " 0
             exiting=1
             exit 1
        }
        visited[0]=1
    }
    else {
        movdirX = (x - prevX)/2
        movdirY = (y - prevY)/2
        if(mapref[prevY+movdirY+(nlines+1)/2][prevX+movdirX+(linelength+1)/2]==" ") {
             print "prevY",prevY, "prevX", prevX ,"movY", movdirY ,"movX", movdirX
             print "Error in path"
             print "Planning score: " 0
             exiting=1
             exit 1
        }
        if(mapref[y+(nlines+1)/2][x+(linelength+1)/2] ~ /[0-9]/) {
            visited[mapref[y+(nlines+1)/2][x+(linelength+1)/2]+0] = 1
        }
        robDist += sqrt(4*movdirX*movdirX+4*movdirY*movdirY)
    }


    prevX = x
    prevY = y

    pathind++

}

END {
#    print "map"
#    for(l=27;l>=1;l--) {
#       for (c=1; c<=55; c++) {
#           printf("%s",mapref[l][c]) 
#       }
#       print ""
#    }

     if(exiting>0) {
         exit exiting
     }

     if(x!=0 || y!=0) {
           print "Path does not finish at 0 0"
           print "Planning score: " 0
           exit 1
     }

     nTargets=0
     nonVisitedTargets=0
     for (t in visited) {
         if (visited[t]!=1) {
            print "Target " t " not visited"
            nonVisitedTargets++
         }
         nTargets++
     }

     if (robDist==0) {
	     score=0
     }
     else {
        #robDist = (pathind-2)*2
        #if(robDist < bestDist) robDist=bestDist
	#print "robDist " robDist " bestDist " bestDist " nonVisitedTargets " nonVisitedTargets " nTargets " nTargets 
        score = bestDist/robDist - nonVisitedTargets/nTargets
	if (score > (nTargets - nonVisitedTargets)/nTargets) {
            score = (nTargets - nonVisitedTargets)/nTargets
        }

     }
     print "Planning score: " score
}
