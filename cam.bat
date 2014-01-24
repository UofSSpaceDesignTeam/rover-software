@echo off
cd winmp
nc.exe -l -p 3001 | mplayer.exe -x 880 -y 415 -nosound -noborder -hardframedrop -noautosub -quiet -fps 40 -priority high -ontop -geometry 165:30 -demuxer h264es -nocache -"