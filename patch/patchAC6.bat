@echo off

xcopy /E /Y %~dp0\AC6\* %~dp0\..\
msg * "current project support AC6"
