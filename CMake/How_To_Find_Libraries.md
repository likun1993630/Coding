# 待翻译

## How To Find Libraries

If your software uses external libraries (i.e. libraries not coming with your software), you don't know in advance where its headers and libraries will be located on the system where your software will be compiled. Depending on the location appropriate include directories and library search paths will have to be added to the compile commands.

CMake helps you with this by providing the [find_package](http://www.cmake.org/cmake/help/cmake-2-8-docs.html#command:find_package) command.

This article briefly discusses how to use external libraries in a CMake project and then moves on to how to write your own find modules for libraries that don't already have one.

#  Using external libraries

CMake comes with numerous *modules* that aid in finding various well-known libraries and packages. You can get a listing of which modules your version of CMake supports by typing *cmake --help-module-list*, or by figuring out where your modules-path is and looking inside of it. On Ubuntu linux, for example, the module path is */usr/share/cmake-3.5/Modules/*

Let's take the very popular *bzip2* library. There is a module named *FindBZip2.cmake*. Once you call this module with find_package(BZip2), cmake will automatically fill in the values of various variables, which you can then use in your CMake script. For a list of the variables, look at the actual cmake module file, or you can type *cmake --help-module FindBZip2*.

```shell
➜ ~ cmake --help-module FindBZip2
FindBZip2
---------

Try to find BZip2

Once done this will define

::

 BZIP2_FOUND - system has BZip2
 BZIP2_INCLUDE_DIR - the BZip2 include directory
 BZIP2_LIBRARIES - Link these to use BZip2
 BZIP2_NEED_PREFIX - this is set if the functions are prefixed with BZ2_
 BZIP2_VERSION_STRING - the version of BZip2 found (since CMake 2.8.8)
```

For example, consider a very simple program that uses *bzip2* - i.e. the compiler needs to know where *bzlib.h* is and the linker needs to find the bzip2 library (for dynamic linking, this is something like *libbz2.so* on Unix, or *libbz2.dll* on Windows):

```cmake
cmake_minimum_required(VERSION 2.8)
project(helloworld)
add_executable(helloworld hello.c)
find_package (BZip2)
if (BZIP2_FOUND)
  include_directories(${BZIP_INCLUDE_DIRS})
  target_link_libraries (helloworld ${BZIP2_LIBRARIES})
endif (BZIP2_FOUND)
```

You can then use *cmake* and *make VERBOSE=1* to verify that the correct flags are being passed to the compiler and linker. You can also use a tool like `ldd` or `dependency walker` after compilation to verify what exact files *helloworld* has been linked with.

#  Using external libraries that CMake doesn't yet have modules for

Suppose you want to use the LibXML++ library. As of this writing, CMake doesn't come with a 'find' module for libXML++. But you found one on the internet (in a file called *FindLibXML++.cmake*) by googling for *FindLibXML++ Cmake*. In CMakeLists.txt, write

```cmake
find_package(LibXML++ REQUIRED)
include_directories(${LibXML++_INCLUDE_DIRS})
set(LIBS ${LIBS} ${LibXML++_LIBRARIES})
```

If the package is optional, you can omit the `REQUIRED` keyword and query the boolean variable `LibXML++_FOUND` afterwards to see if it has been found. Then, after detecting all the libraries, for your target:

```cmake
target_link_libraries(exampleProgram ${LIBS})
```

For this to work, you'll need to put the *FindLibXML++.cmake* file into your CMake module path. Since CMake (currently) doesn't ship it, you'll have to ship it within your project. Create a folder named cmake/Modules/ under your project root, and in the root CMakeLists.txt, include the following code:

```cmake
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
```

As you may have guessed, you need to put the CMake modules that you use, and that CMake has to find automatically, inside that folder.

That's it, usually. Some libraries might require something else, so be sure to look inside that FindSomething.cmake file to see the documentation for that specific library.

##  Packages with components

Some libraries are not monolithic, but come with one or more dependent libraries or components. A notable example for this is the Qt library, which ships (among others) with the components QtOpenGL and QtXml. To use both of these components, use the following the find_package command:

```cmake
find_package(Qt COMPONENTS QtOpenGL QtXml REQUIRED)
```

Again, you can omit the `REQUIRED` keyword, if the package is optional for your project. In this case, you can use the `_``_FOUND` variable (e.g. Qt_QtXml_FOUND) to check if a component has been found. The following invocations of find_package are all equivalent:

```cmake
find_package(Qt COMPONENTS QtOpenGL QtXml REQUIRED)
find_package(Qt REQUIRED COMPONENTS QtOpenGL QtXml)
find_package(Qt REQUIRED QtOpenGL QtXml)
```

If you only require some components of a package, and want to use others only, if they are available, you can call find_package twice:

```cmake
find_package(Qt COMPONENTS QtXml REQUIRED) # QtXml是必需的，找不到无法编译
find_package(Qt COMPONENTS QtOpenGL) # QtOpenGL 不是必须，找不到也可以继续编译
```

Alternatively, you can invoke find_package once with all components, but without the `REQUIRED` keyword and then explicitly check the required components:

```cmake
find_package(Qt COMPONENTS QtOpenGL QtXml)
if ( NOT Qt_FOUND OR NOT QtXml_FOUND )
  message(FATAL_ERROR "Package Qt and component QtXml required, but not found!")
endif( NOT Qt_FOUND OR NOT QtXml_FOUND )
```

#  How package finding works

The [find_package()](http://www.cmake.org/cmake/help/cmake-2-8-docs.html#command:find_package) command will look in the module path for Find.cmake, which is the typical way for finding libraries. First CMake checks all directories in ${CMAKE_MODULE_PATH}, then it looks in its own module directory <CMAKE_ROOT>/share/cmake-x.y/Modules/.

If no such file is found, it looks for Config.cmake or -config.cmake, which are supposed to be installed by libraries (but there are currently not yet many libraries which install them) and that don't do detection, but rather just contain hardcoded values for the installed library.

The former is called module mode and the latter is called config mode. Creation of config mode files is documented [here](https://gitlab.kitware.com/cmake/community/wikis/doc/tutorials/doc/tutorials/How-to-create-a-ProjectConfig.cmake-file). You may also need the documentation for [importing and exporting targets](https://gitlab.kitware.com/cmake/community/wikis/doc/tutorials/Exporting-and-Importing-Targets).

For the module system there seems to be no documentation elsewhere, so this article concentrates on it.

No matter which mode is used, if the package has been found, a set of variables will be defined:

- _FOUND
- _INCLUDE_DIRS or _INCLUDES
- _LIBRARIES or _LIBRARIES or _LIBS
- _DEFINITIONS

All this takes place in the Find.cmake file.

Now, in the CMakeLists.txt file in the top level directory of your code (the client code that is actually going to make use of the library , we check for the variable _FOUND to see whether the package has been found or not. For most packages the resulting variables use the name of the package all uppercased, e.g. LIBFOO_FOUND, for some packages the exact case of the package is used, e.g. LibFoo_FOUND. If this variable is found, then, we pass the_INCLUDE_DIRS to the include_directories() command and _LIBRARIES to the target_link_libraries() command of CMake.

These conventions are documented in the file readme.txt in the CMake module directory.

The "REQUIRED" and other optional find_package arguments are forwarded to the module by find_package and the module should affect its operation based on them.

#  Piggybacking on pkg-config

Pkg-config is a build-helping tool, based on '.pc' files that record the location of library-files and include-files. It is typically found on Unix-like systems. Please see [the pkg-config’s site](http://www.freedesktop.org/wiki/Software/pkg-config/) for more information. CMake has its own functions that can make use of pkg-config if it is installed on your system. The functions are documented in the `FindPkgConfig.cmake` file under your CMake’s Modules directory. This can be particularly helpful if you are dealing with a library that does not have a cmake-script built for it, or if you are dealing with a situation where CMake’s ordinary find-script does not work properly.

However, you should be very careful if you just call pkg-config and use whatever it returned, even when it is available. One major reason for this is that this way the user can accidently override or augment library detection by manually defining the paths using ccmake, as is conventional with CMake. There are also cases when pkg-config just supplies incorrect information (wrong compiler, etc). In these cases, let CMake do the detection, as it would without pkg-config, but use pkg-config to provide additional hints on where to look.

#  Writing find modules

First of all, notice that the name, or prefix, supplied to find_package is part of the filename and the prefix used for all variables. The case does matter and the names should match exactly. Unfortunately in many cases, even in the modules shipped with CMake, the names do not match, causing various issues.

The basic operation of a module should roughly follow this order

- Use find_package to detect other libraries that the library depends on 
  - The arguments QUIETLY and REQUIRED should be forwarded (e.g. if current package was REQUIRED, the depencency should also be)
- Optionally use pkg-config to detect include/library paths (if pkg-config is available)
- Use find_path and find_library to look for the header and library files, respectively 
  - Paths supplied by pkg-config are used only as hints on where to look
  - CMake has many other hardcoded locations where it looks, too
  - Results should be saved in variables _INCLUDE_DIR and _LIBRARY (note: singular, not plural)
- Set _INCLUDE_DIRS to _INCLUDE_DIR _INCLUDE_DIRS ...
- Set _LIBRARIES to _LIBRARY _LIBRARIES ... 
  - Dependencies use plural forms, the package itself uses the singular forms defined by find_path and find_library
- Call the find_package_handle_standard_args() macro to set the _FOUND variable and print a success or failure message.

```cmake
# - Try to find LibXml2
# Once done this will define
#  LIBXML2_FOUND - System has LibXml2
#  LIBXML2_INCLUDE_DIRS - The LibXml2 include directories
#  LIBXML2_LIBRARIES - The libraries needed to use LibXml2
#  LIBXML2_DEFINITIONS - Compiler switches required for using LibXml2

find_package(PkgConfig)
pkg_check_modules(PC_LIBXML QUIET libxml-2.0)
set(LIBXML2_DEFINITIONS ${PC_LIBXML_CFLAGS_OTHER})

find_path(LIBXML2_INCLUDE_DIR libxml/xpath.h
          HINTS ${PC_LIBXML_INCLUDEDIR} ${PC_LIBXML_INCLUDE_DIRS}
          PATH_SUFFIXES libxml2 )

find_library(LIBXML2_LIBRARY NAMES xml2 libxml2
             HINTS ${PC_LIBXML_LIBDIR} ${PC_LIBXML_LIBRARY_DIRS} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibXml2  DEFAULT_MSG
                                  LIBXML2_LIBRARY LIBXML2_INCLUDE_DIR)

mark_as_advanced(LIBXML2_INCLUDE_DIR LIBXML2_LIBRARY )

set(LIBXML2_LIBRARIES ${LIBXML2_LIBRARY} )
set(LIBXML2_INCLUDE_DIRS ${LIBXML2_INCLUDE_DIR} )
```

In the first line, LibFindMacros is included. For this to work, the LibFindMacros.cmake file must be placed in the module path, as it is not currently shipped in the CMake distribution.

##  Finding files

After that the actual detection takes place. Supply a variable name as the first argument for both find_path and find_library. If you need multiple include paths, use find_path multiple times with different variable names. The same goes for find_library.

- NAMES specifies one or more names for the target and if any matches, that one is chosen. In find_path you should use the main header or whatever is #included in the C/C++ code. This may also contain a folder, e.g. alsa/asound.h and then it will give the parent directory of the folder that asound.h is in as result.
- PATHS is used to provide additional search paths for CMake to look into and it should not generally be defined for things other than pkg-config (CMake has its built-in defaults and more can be added as required by various config variables). If you are not using it, leave out the entire section.
- PATH_SUFFIXES (not present in this example) is useful for libraries that on some systems put their files in paths like /usr/include/ExampleLibrary-1.23/ExampleLibrary/main.h; in this case you'd use NAMES ExampleLibrary/main.h PATH_SUFFIXES ExampleLibrary-1.23. Multiple suffixes may be specified and CMake will try them all, and also no suffix at all, in all include directories, and in the bare prefix as well.

The library names do not contain the lib prefix used on UNIX system, nor any file extension or compiler specifications, as CMake will platform-independently detect them. The library version number is still needed, if present in the body part of the library file name.

##  Using LibFindMacros

The module [LibFindMacros.cmake](https://github.com/Tronic/cmake-modules/blob/master/LibFindMacros.cmake) was written to make writing find-modules easier. It contains various libfind macros taking care of the boring parts that are always the same for every library. With it, a scripts look like this:

```cmake
# - Try to find ImageMagick++
# Once done, this will define
#
#  Magick++_FOUND - system has Magick++
#  Magick++_INCLUDE_DIRS - the Magick++ include directories
#  Magick++_LIBRARIES - link these to use Magick++

include(LibFindMacros)

# Dependencies
libfind_package(Magick++ Magick)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Magick++_PKGCONF ImageMagick++)

# Include dir
find_path(Magick++_INCLUDE_DIR
  NAMES Magick++.h
  PATHS ${Magick++_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Magick++_LIBRARY
  NAMES Magick++
  PATHS ${Magick++_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Magick++_PROCESS_INCLUDES Magick++_INCLUDE_DIR Magick_INCLUDE_DIRS)
set(Magick++_PROCESS_LIBS Magick++_LIBRARY Magick_LIBRARIES)
libfind_process(Magick++)
```

libfind_pkg_check_modules is a convenience wrapper for CMake's own pkg-config modules, with the intention of making things easier. You don't need to test for CMake version, load the appropriate module, check if it loaded, etc. when all you really want to do is a simple optional check. The arguments are the same as for [pkg_check_modules](http://www.cmake.org/cmake/help/cmake2.6docs.html#module:FindPkgConfig): first the prefix for returned variables, then package name (as it is known by pkg-config). This defines _INCLUDE_DIRS and other such variables.

###  Dependencies (optional)

libfind_package functions similarly to find_package, except that it forwards the QUIETLY and REQUIRED arguments. For this to work, the first parameter supplied is the name of the current package. I.e. here Magick++ depends on Magick. Other arguments such as version could be added after Magick and they'd just be forwarded to the CMake internal find_package command. Have one of these lines for every library that your library depends on, and be sure to supply find modules for them as well.

###  Final processing

Three items done, four to go. Fortunately, those last ones are rather mechanical and can be taken care of by the libfind_process macro and the last three lines of the example file. You will need to set _PROCESS_INCLUDES with the names of all variables to be included in _INCLUDE_DIRS, and _PROCESS_LIBS with the names of all variables to be included in _LIBRARIES. Then call libfind_process() and it'll do the rest.

The library is considered FOUND only if all the provided variables have valid values.

#  Performance and caching

The CMake variable system is far more complex than it may seem first. Some variables are cached and some are not. The cached variables may be cached as internal (not possible to edit with ccmake) or as external (have a type and a documentation string and can be modified in ccmake). Further, the external variables may be set advanced, so that they'll only be seen in the advanced mode of ccmake.

By default, all variables are non-cached.

In order to avoid doing all the library detection again on every run, and more importantly to allow the user to set include dirs and libraries in ccmake, these will have to be cached. Fortunately, this is already taken care of by find_path and find_library, which will cache their variables. If the variable is already set to a valid value (e.g. not -NOTFOUND or undefined), these functions will do nothing, but keep the old value. Similarly, pkg_check_modules does internal caching of the results, so that it does not need to call pkg-config again every time.

On the other hand, the output values of the find module (_FOUND, _INCLUDE_DIRS and _LIBRARIES) should not be cached because then modifying the other cached variables would no longer cause the actual output to change and this obviously is not a desired operation.

#  Common bugs in find modules

- Different case or slightly different name in filename and in variables
- The module does not check _FIND_REQUIRED or _FIND_QUIETLY - and thus the find_package arguments QUIET and REQUIRED will have no effect
- _INCLUDE_DIRS and _LIBRARIES are not set, but instead only the singular forms are available

#  Links

- [Tronic's CMake modules](http://zi.fi/cmake/Modules)

------

This page was initially populated by conversion from its [original location](https://public.kitware.com/Wiki/CMake:How_To_Find_Libraries) in another wiki.



# 原文：

https://gitlab.kitware.com/cmake/community/wikis/doc/tutorials/How-To-Find-Libraries