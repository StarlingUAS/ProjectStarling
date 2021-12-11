# Introduction to Starling with the Fenswood Scenario

This Tutorial aims to give an introductory overview of the Starling system through the use of the Fenswood Scenario example. It will take a student from the basics of Linux, ROS and Docker to hopefully being able to run the Fenswood Scenario simulation environment with a simple controller flying the drone.

This tutorial was written for the University of Bristol Aerial MSc Group Project 2021.

**On this page:**

[TOC]

## A Brief Introduction to Linux

Adapted from this [digital ocean tutorial](https://www.digitalocean.com/community/tutorials/an-introduction-to-linux-basics)

### What is Linux
Linux is a family of free and open-source operating systems based on the Linux kernel (core operating system). Operating systems based on Linux are known as Linux distributions or distros. Examples include Debian, Ubuntu, Fedora, CentOS, Gentoo, Arch Linux, and many others.

The Linux kernel has been under active development since 1991, and has proven to be extremely versatile and adaptable. You can find computers that run Linux in a wide variety of contexts all over the world, from web servers to cell phones. Today, 90% of all cloud infrastructure and 74% of the world’s smartphones are powered by Linux.

However, newcomers to Linux may find it somewhat difficult to approach, as Linux filesystems have a different structure than those found on Windows or MacOS. Additionally, Linux-based operating systems depend heavily on working with the command line interface, while most personal computers rely on graphical interfaces.

### The Terminal

The terms “terminal,” “shell,” and “command line interface” are often used interchangeably, but there are subtle differences between them:

* A **terminal** is an input and output environment that presents a text-only window running a shell.
* A **shell** is a program that exposes the computer’s operating system to a user or program. In Linux systems, the shell presented in a terminal is a command line interpreter. The default shell in Ubuntu Linux is known as `bash`.
* A **command line interface** is a user interface (managed by a command line interpreter program) which processes commands to a computer program and outputs the results.

When someone refers to one of these three terms in the context of Linux, they generally mean a terminal environment where you can run commands and see the results printed out to the terminal, such as this:

![linux terminal](imgs/linux-terminal-on-ubuntu.png)

There are two ways to open a terminal:

1. Pressing the ++win++ or ++cmd++ key to open the program menu and typing `terminal`, then pressing ++enter++
2. Pressing ++ctrl+alt+t++

This default terminal is known as the 'gnome-terminal'. Other terminals exist such as ['terminator'](https://terminator-gtk3.readthedocs.io/en/latest/)

Becoming a Linux user requires you to be comfortable with using a terminal. Any administrative task, including file manipulation, package installation, and user management, can be accomplished through the terminal. The terminal is interactive: you specify commands to run (after the $ sign) and the terminal outputs the results of those commands. To execute any command, you type it into the prompt and press ++enter++.

When using the Starling system, interacting with Docker and ROS2, you'll most often be doing so through a terminal shell. Although personal computers that run Linux often come with the kind of graphical desktop environment familiar to most computer users, it is often more efficient or practical to perform certain tasks through commands entered into the terminal. As of writing, a GUI (Graphical User Interface) has not been developed for Starling, and so almost all tasks have to be achieved through the terminal shell.

A basic command to try out is `echo`, which will print things to the terminal. For example `echo hello-world` will print `hello-world` into the terminal. You can also use it to observe the value of **Environment Variables** which record and keep useful variables to the operation of the Operating System. For example, when you run a command in `bash`, `bash` will look for the command executable in the locations provided by the environment variable `PATH`. You can print the contents of this env-var using `echo $PATH`. The `$` before the name of the variable tells `bash` that the following word represents an environment variable, and that it should be looked up.

### Navigating the file system

Like Windows and Mac, the Linux filesystems are based on a directory tree. This means that you can create directories (which are functionally identical to folders found in other operating systems) inside other directories, and files can exist in any directory.

The forward slash (`/`) is used to indicate the root directory in the filesystem hierarchy.

When a user logs in to the shell, they are brought to their own user directory, stored within `/home/<username>`. This is referred to as the user’s home directory. Often you may see the *tilde* (`~`) character when specifying a file location (e.g. `~/Documents/hello.txt` = `/home/<username>/Documents/hello.txt`). This is shorthand for the user's home directory and gets substituted in when used.

To see what directory you are currently active in you can run the `pwd` command, which stands for “print working directory”
```console
myuser@my-machine:~$ pwd
/home/myuser
```

To see a list of files and directories that exist in your current working directory, run the `ls` command:
```console
myuser@my-machine:~$ ls
Documents Desktop Pictures ...
```
You can get more details if you run `ls -al` command:

```console
myuser@my-machine:~$ ls -al
drwxr-xr-x  2 myuser myuser   4096 Apr 30  2021  Desktop
drwxrwxr-x  8 myuser myuser   4096 Oct 29 09:27  Documents
...
```

You can create one or more new directories within your current working directory with the `mkdir` command, which stands for “make directory”. For example, to create two new directories named testdir1 and testdir2, you might run the first command. You can create nested directories by using the `-p` option:
```console
myuser@my-machine:~$ mkdir testdir1 testdir2
myuser@my-machine:~$ mkdir -p testdir3/testsubdir
```

To navigate into one of these new directories, run the cd command (which stands for “change directory”) and specify the directory’s name:
```console
myuser@my-machine:~$ cd testdir1
myuser@my-machine:~/testdir1$
```

Note that you can navigate from anywhere to anywhere. `cd` only requires a valid filepath. Note also that `.` represents the current folder and `..` represents the parent folder. Note also how is shows the current working directory in the shell as well.
```bash
cd # This will bring you back to home directory
cd testdir3/testsubdir # Brings you into testsubdir
cd ../ # Brings you back out one level into testdir3
cd ../testdir1 # Brings you back out one level and back into testdir1
cd /home/<username>/testdir2 # Absolute reference to testdir2
cd ~/testdir2 # Absolute reference using tilde to testdir2
```

### Working with files

You cannot use cd to interact with files; cd stands for “change directory”, and only allows you to navigate directories. You can, however, create, edit, and view the contents of files.

One way to create a file is with the touch command. This creates an empty file in your current working directory. To create a new file called file.txt:
```bash
touch file.txt
```

If you decide to rename file.txt later on, you can do so with the mv command. mv stands for “move” and it can move a file or directory from one place to another. By specifying the original file, file.txt, you can “move” it to a new location in the current working directory, thereby renaming it.
```bash
mv file.txt newfile.txt
```

It is also possible to copy a file to a new location with the cp command. If we want to bring back file.txt but keep newfile.txt, you can make a copy of newfile.txt named file.txt like this:
```bash
cp newfile.txt file.txt
```

However, files are not of much use if they don’t contain anything. To edit files, a file editor is necessary.
There are many options for file editors, all created by professionals for daily use. Such editors include vim, emacs, nano, and pico.
`nano` is a suitable option for beginners: it is relatively user-friendly and doesn’t overload you with cryptic options or commands.
```bash
nano file.txt
```
This will open a space where you can start typing to edit the file. In `nano` specifically you can save your written text by pressing ++ctrl+x++, ++y++, and then ++enter++. This returns you to the shell with a newly saved `file.txt`.

Now that file.txt has some text within it, you can view it using cat or `less`.

The `cat` command prints the contents of a specified file to your system’s output. Try running `cat` and pass the `file.txt` file you just edited as an argument:
```bash
cat file.txt
```

Using `cat` to view file contents can be unwieldy and difficult to read if the file is particularly long. As an alternative, you can use the `less` command which will allow you to paginate the output. Use `less` to view the contents of the file.txt file, like this:
```bash
less file.txt
```
This will also print the contents of file.txt, but one terminal page at a time beginning at the start of the file. You can use the spacebar to advance a page, or the arrow keys to go up and down one line at a time. Press ++q++ to quit out of `less`.

Finally, to delete the file.txt file, pass the name of the file as an argument to `rm`:
```bash
rm file.txt
rm -d directory
rmidr directory
rm -r directory # If the directory you are deleting is not empty
```

> **NOTE**: If your question has to do with a specific Linux command, the manual pages offer detailed and insightful documentation for nearly every command. To see the `man` page for any command, pass the command’s name as an argument to the `man` command - `man command`.
> For instance, `man rm` displays the purpose of `rm`, how to use it, what options are available, examples of use, and more useful information.

### Connecting the laptop to the Internet (Eduroam)

For University of Bristol students connecting to Eduroam see theese [instructions](https://www.wireless.bris.ac.uk/eduroam/instructions/go-ubuntu/)

### Installing Dependencies and Useful Programs






## Why Starling and Docker

## Running the Example Fenswood Scenario

## A Brief Introduction to ROS

## Running the Example UAV Controller

## Inspecting and Debugging Starling with Docker and ROS