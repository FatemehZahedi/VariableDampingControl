BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

###############################################################
###                    EXTERNAL LIBRARIES                  ####
###############################################################

# ---------------- Boost ----------------------
# BOOST_DIR = /home/tannerbitz/Documents/cpp/boost_1_68_0 	# Tanner's Laptop
BOOST_DIR = /usr/include/boost_1_70_0						# KUKA Computer
CXXFLAGS 	+= 	-std=c++11 -pthread
BOOST_INC 	= 	/usr/local/include $(BOOST_DIR)
BOOST_LIB 	= 	-L/usr/local/lib/ 	\
				-lboost_system 		\
				-lpthread 			\
				-lboost_thread 		\
				-lboost_chrono	\
				-lboost_filesystem

# ---------------- HDF ----------------------
HDF_INSTALL 	= /usr/local/hdf5
HDF_LIB 	= $(HDF_INSTALL)/lib			
HDF_INCLUDE	= $(HDF_INSTALL)/include
HDF_LIBS 	= -L$(HDF_LIB)	\
			  -lhdf5_cpp	\
			  -lhdf5 		\
			  -lz			\
			  -lm 			\
			  -ldl

###############################################################
###  CONCATENATE VARIABLES FOR SOURCES, COMPILER FLAGS,     ###
###  INCLUDE DIRECTORIES, AND LIBRARY DIRECTORIES FOR       ###
###  COMPILING            								    ###
###############################################################

SOURCES 	= KukaVariableDamping.cpp	\
			  PositionControlClient.cpp	\
			  UdpServer.cpp				\
			  TrignoEmgClient.cpp		

TARGET 		= KukaVariableDamping

CXXFLAGS 		+= -std=c++11 -pthread
LDFLAGS 		+= $(LIB_DIR)/libFRIClient.a
LDFLAGS			+= $(HDF_LIBS)
LDFLAGS			+= $(BOOST_LIB)
INC_DIR 		+=/usr/include/	\
					$(HDF_INCLUDE)	\
					$(BOOST_INC)


###############################################################
###                        COMPILE                         ####
###############################################################
include $(BASE_DIR)/build/GNUMake/rules_examples.mak
