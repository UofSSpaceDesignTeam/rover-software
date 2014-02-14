@echo off
cd winmp
nc.exe -l -p 3001 | mplayer.exe -really-quiet -xy 0.5 -nosound -noborder -hardframedrop -noautosub -fps 40 -priority high -ontop -geometry 160:30 -demuxer h264es -nocache -"