
BEGIN {
   FS=""
   line=1;
   margin=2
   discount=0.85
   linelength=49
   nlines=21
   for(di=-margin; di<=margin; di++) {
        for(dj=-margin; dj<=margin; dj++) {
            score[di][dj]=0;
        }
   }
}


function abs(v) {v += 0; return v < 0 ? -v : v}
function isnum(x)   { return x+0 == x }


# to read reference map to array a
FNR==NR {
   a[NR]=$0

   next
}

{
     nmap=split($0,map,"") 

     if (length(map)!=linelength) {
         print "length of line " line " is not correct!"
     }

     for(di=-margin; di<=margin; di++) {
         if ((line+di) < 0) continue
         nmapref=split(a[line+di],mapref,"") 
         # to allow to have planning.out as input to this script (planning.out has the map, with linelength columns, but it also has other stuff)
         if (length(a[line+di])!=linelength) {
            continue
         }
         for(dj=-margin; dj<=margin; dj++) {
             #if(di==0 && dj==0) print "line " line
             for(i=1;i<=length(a[line+di]);i++) {
                 if ((i+dj)<0) continue
                 if ((i+dj)>=length(map)) continue
                 if(map[i+dj]!=" ") {
                      if(mapref[i]==map[i+dj]) {
                           score[di][dj]+=10  # correctly mapped
                           #print "+10"
                      }
                      else { 
                           score[di][dj]-=5   # uncorrectly mapped
                           #print "-5"
                      }
                 }
             }
             #if(di==0 && dj==0 ) print "score " score[0][0]
         }
     }
     line++
}

END {
    if (line!=nlines+1) {
         print "number of lines is not correct!"
    }
    scoremax=-200000
    for(di=-margin; di<=margin; di++) {
         for(dj=-margin; dj<=margin; dj++) {
             #print "mapping score " di " " dj " :" score[di][dj]
             if (score[di][dj] > scoremax) {
                 scoremax = score[di][dj]
                 distmax = abs(di)+abs(dj)
             }
                 
          }
    }
    print "mapping score:" (scoremax)*discount^(distmax) 
}
