Using Visual 2017 with Makefile for Linux Issues

Does not include files from remote file system - looks to local store

adjust cmakesettings.json 
``` json
// See https://go.microsoft.com//fwlink//?linkid=834763 for more information about this file.
  "configurations": [
    {
      "name": "Linux-Debug",
      "generator": "Unix Makefiles",
      "remoteMachineName": "${defaultRemoteMachineName}",
      "configurationType": "Debug",
      "remoteCMakeListsRoot": "/var/tmp/src/${workspaceHash}/${name}",
      "cmakeExecutable": "/usr/local/bin/cmake",
      "buildRoot": "${env.USERPROFILE}\\CMakeBuilds\\${workspaceHash}\\build\\${name}",
      "installRoot": "${env.USERPROFILE}\\CMakeBuilds\\${workspaceHash}\\install\\${name}",
      "remoteBuildRoot": "/var/tmp/build/${workspaceHash}/build/${name}",
      "remoteInstallRoot": "/var/tmp/build/${workspaceHash}/install/${name}",
      "remoteCopySources": true,
      "remoteCopySourcesOutputVerbosity": "Normal",
      "remoteCopySourcesConcurrentCopies": "10",
      "remoteCopySourcesMethod": "sftp",
      "cmakeCommandArgs": "",
      "buildCommandArgs": "",
      "ctestCommandArgs": "",
      "inheritEnvironments": [ "linux-x64" ]
    },
    {
      "name": "Linux-Release",
      "generator": "Unix Makefiles",
      "remoteMachineName": "${defaultRemoteMachineName}",
      "configurationType": "Release",
      "remoteCMakeListsRoot": "/var/tmp/src/${workspaceHash}/${name}",
      "cmakeExecutable": "/usr/local/bin/cmake",
      "buildRoot": "${env.USERPROFILE}\\CMakeBuilds\\${workspaceHash}\\build\\${name}",
      "installRoot": "${env.USERPROFILE}\\CMakeBuilds\\${workspaceHash}\\install\\${name}",
      "remoteBuildRoot": "/var/tmp/build/${workspaceHash}/build/${name}",
      "remoteInstallRoot": "/var/tmp/build/${workspaceHash}/install/${name}",
      "remoteCopySources": true,
      "remoteCopySourcesOutputVerbosity": "Normal",
      "remoteCopySourcesConcurrentCopies": "10",
      "remoteCopySourcesMethod": "sftp",
      "cmakeCommandArgs": "",
      "buildCommandArgs": "",
      "ctestCommandArgs": "",
      "inheritEnvironments": [ "linux-x64" ]
    },
    {
      "name": "x64-Release",
      "generator": "Ninja",
      "configurationType": "RelWithDebInfo",
      "inheritEnvironments": [ "msvc_x64_x64" ],
      "buildRoot": "${env.USERPROFILE}\\CMakeBuilds\\${workspaceHash}\\build\\${name}",
      "installRoot": "${env.USERPROFILE}\\CMakeBuilds\\${workspaceHash}\\install\\${name}",
      "cmakeCommandArgs": "",
      "buildCommandArgs": "-v",
      "ctestCommandArgs": ""
    }
  ]
}
```

and CppProperties.json to
```json
{
  "configurations": [
    {
      "inheritEnvironments": [
        "linux-gcc-x64"
      ],
      "name": "Linux-Debug",
      "includePath": [
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include\\c++\\5",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include\\x86_64-linux-gnu\\c++\\5",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include\\c++\\5\\backward",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\lib\\gcc\\x86_64-linux-gnu\\5\\include",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\local\\include",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\lib\\gcc\\x86_64-linux-gnu\\5\\include-fixed",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include\\x86_64-linux-gnu",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include"
      ],
      "defines": [
        "_DEBUG",
        "UNICODE",
        "_UNICODE"
      ],
      "intelliSenseMode": "linux-gcc-x64"
    },
    {
      "inheritEnvironments": [
        "msvc_x64"
      ],
      "name": "x64-Release",
      "includePath": [
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include\\c++\\5",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include\\x86_64-linux-gnu\\c++\\5",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include\\c++\\5\\backward",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\lib\\gcc\\x86_64-linux-gnu\\5\\include",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\local\\include",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\lib\\gcc\\x86_64-linux-gnu\\5\\include-fixed",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include\\x86_64-linux-gnu",
        "C:\\Users\\tknight\\AppData\\Local\\lxss\\rootfs\\usr\\include"
      ],
      "defines": [
        "WIN32",
        "NDEBUG",
        "UNICODE",
        "_UNICODE"
      ],
      "intelliSenseMode": "windows-msvc-x64"
    }
  ]
}
```