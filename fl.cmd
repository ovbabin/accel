setlocal
set "PATH=C:\Programs\stlink-1.7.0-x86_64-w64-mingw32\bin;%PATH%"
st-flash --format ihex write build/accel.hex
endlocal
