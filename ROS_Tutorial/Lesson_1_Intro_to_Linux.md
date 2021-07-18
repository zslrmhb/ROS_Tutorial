# Lesson 1 - Intro to Linux


## *What is Linux*
>**This is a family of operating system based on a open-source operating system kernal. ROS was based on this system, so let's first get familiarize with it!**

## *Common Linux Commands*


**`Ctrl + Alt + T`**
>**open the terminal**
Usage: pwd

**`pwd`**  
>**current working directory**
Usage: pwd

**`cd`**
>**change to directory**
Usage: cd [pathname]

**`ls`**
>**list directory contents**
Usage: ls (optional flags: -l(ong) , -a(ll))

**`clear`**
>**clean / refresh terminal**
 Usage: clear

**`touch`** 
>**create file**
Usage: touch [file name]

**`mv`**
>**move / rename files**

**`mkdir`**
>**make directory**
Usage: mkdir [directory name]

**`rmdir`**
>**remove empty directory**
Usage: rmdir [directory name]

**`rm`**
>**remove directories or contents**
>**WARNING: AVOID USING rm -f as much as possible, SEARCH it up if you are curious**

**`cp`**
>**copy files and directories**

**`chmod`**
>**change permissions of files and directories**

**`echo`**
>**output the string contains in the arguments**
Usage: echo [string arguments]

**`sudo`**
>**enables administrative permissions**
Usage: sudo [command]
---

## Practices
**1) Open the terminal and create a folder called "test"**

**2) How will you check if the "test" folder exist?**

**3) change your directory to "test"**

**4) check your current directory**

**5) In "test", create a file called "Hello_World", and check if it is created**

**6) make a new directory called "test2"**

**7) move "Hello_World" to "test2"**

**8) change to directory "test2" and rename "Hello_World" file to "Hello"**

**9) Copy the "Hello" file to the previous directory, and remove the "Hello" in the current directory**

**10) Back to the previous directory and remove directory "test2"**

**11) CHALLENGE: move to the previous directory and delete the directory "test" (WITH the file "Hello" in "test"). Then, echo the following:  "Finished!"**

## Answer References
1) first, press Ctrl + Alt + T
```bash
$ mkdir test
```
2) 
```bash
$ ls
```
3) 
```bash
$ cd test 
```
4) 
```bash
$ pwd
/home/ucsd/test
```
5) 
```bash
$ touch Hello_World
$ ls
Hello_World
```
6)
```bash
$ mkdir test2
```
7)
```bash
$ mv Hello_World test2
```
8)
```bash
$ cd test2
$ mv Hello_World Hello
```
9)
```bash
$ cp Hello ..
$ rm Hello
```
10)
```bash
$ cd ..
$ rmdir test2
```
11)
```bash
$ cd ..
$ rm -r test    # This means to remove recursively 
                # For situation when folder is not empty
$ echo "Finished!"
```
##### *References*
1. [Common Linux Commands](https://www.dummies.com/computers/operating-systems/linux/common-linux-commands/)
2. [35 Linux Basic Commands Every User Should Know](https://www.hostinger.com/tutorials/linux-commands)