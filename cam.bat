@echo off
cd winmp
nc.exe -l -p 3001 | mplayer.exe -x 850 -y 400 -nosound -hardframedrop -noautosub -fps 35 -ontop -geometry 50%:50 -demuxer h264es -nocache -"