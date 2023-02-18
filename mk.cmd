setlocal
set "PATH=%C:\Programs\gcc-arm-none-eabi-10.3-2021.10\bin;C:\Programs\msys64\usr\bin;%PATH%"
make %* 2>&1 | tee .buildlog.txt
endlocal
