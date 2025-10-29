@echo off

xcopy /E /Y %~dp0\AC5\* %~dp0\..\
msg * "current project support AC5"
