@echo OFF
set gitclean=0
set versionfile=../Source/Version.h
for /F %%i in ('git rev-parse --short HEAD') do ( set commitid=%%i)
for /F %%j in ( 'git status ^|findstr clean' ) do (set gitclean=1)   

echo //this is auto generate by the build process, don't modify >%versionfile%
echo #define BUILD_NUMBER (0x%commitid%) >>%versionfile%
echo #define GIT_CLEAN (%gitclean%) >>%versionfile%


