set datafile separator "\t"
# set key outside
set xlabel "Time"
set ylabel "Values"

plot "data.csv" using 1:3 with lines title "over", \
     "data.csv" using 1:4 with lines title "SP", \
     "data.csv" using 1:5 with lines title "Actual"
