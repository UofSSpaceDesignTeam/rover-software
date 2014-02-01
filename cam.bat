@echo off
cd winmp
nc.exe -l -p 3001 | mplayer.exe -x 960 -y 540 -nosound -noborder -hardframedrop -noautosub -quiet -fps 40 -priority high -ontop -geometry 155:30 -demuxer h264es -nocache -"