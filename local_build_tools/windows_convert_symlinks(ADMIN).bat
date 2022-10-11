@ECHO OFF
SET Targetpath=..\.antora\modules\sensor-model
SET Symlinkroot=..\..\..

@REM mklink /D %Targetpath%\images %Symlinkroot%\doc\images

mklink /D %Targetpath%\pages %Symlinkroot%\doc

@REM mklink /D %Targetpath%\partials %Symlinkroot%\_additional_content

@REM mklink /D %Targetpath%\attachments %Symlinkroot%\_attachments

PAUSE