#include "TrignoEmgClient.h"
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <thread>

using namespace H5;

/* Public Functions */
TrignoEmgClient::TrignoEmgClient(std::string ipAddr){
	/* Set IP address */
    _ipAddr = ipAddr;

    /* Create IP address */
    _addr = boost::asio::ip::make_address(_ipAddr);

    /* Set Endpoint Address and Port*/
    _endpointComm.address(_addr);
    _endpointData.address(_addr);
    _endpointComm.port(_portComm);
    _endpointData.port(_portData);


	/* Initialize communicaion port commands */
	_cmds[1] = "START\r\n\r\n";
	_cmds[2] = "STOP\r\n\r\n";
}

/* Public Functions */
TrignoEmgClient::TrignoEmgClient(){
	/* Initialize communicaion port commands */
	_cmds[1] = "START\r\n\r\n";
	_cmds[2] = "STOP\r\n\r\n";
}

void TrignoEmgClient::SetIpAddress(std::string ipAddr){
    /* Set IP address */
    _ipAddr = ipAddr;

    /* Create IP address */
    _addr = boost::asio::ip::make_address(_ipAddr);

    /* Set Endpoint Address and Port*/
    _endpointComm.address(_addr);
    _endpointData.address(_addr);
    _endpointComm.port(_portComm);
    _endpointData.port(_portData);
}

TrignoEmgClient::~TrignoEmgClient(){
    /* If writing, complete last write */
    if (_writeFlag){
        StopWriting();
    }
    /* If receiving data, stop receiving */
    if (_connectedDataPort){
        StopReceiveDataStream();
    }

    /* Close hdf5 variables and free memory if necessary */
    mspace.close();
    datasetEmg.close();
}

void TrignoEmgClient::SetEmgToSave(int emgList[], int nEmgSensors){
    try{
        _emgList = new int[nEmgSensors];
        for (int i=0; i<nEmgSensors; i++){
            _emgList[i] = emgList[i];
        }
        _nActiveEmgSensors = nEmgSensors;
    }
    catch(std::exception & e){
        delete _emgList;
        _emgList = NULL;
        _nActiveEmgSensors = _nSensors;
        printf("Error while Setting Emg List: %s\n", e.what());
    }
}


void TrignoEmgClient::ConnectDataPort(){
    /*
     * This function connects the data port
     */
    try{
        _sockData.connect(_endpointData);
        printf("Data port connected\n");
        _connectedDataPort = true;
    }
    catch(std::exception& e){
        printf("Exception occured while connecting to Trigno Data Port\n");
        printf("Address: %s\n", _ipAddr.c_str());
        printf("Port: %d\n", _endpointData.port());
        printf("Exception: %s\n", e.what());
        _connectedDataPort = false;
    }
}

void TrignoEmgClient::ConnectCommPort(){
 	/*
     * This function connects the comm port
     */
	try{
        _sockComm.connect(_endpointComm);
        printf("Comm socket connected\n");
        _connectedCommPort = true;
        this->GetReplyComm();
    }
    catch(std::exception& e){
        printf("Exception occured while connecting to Trigno Comm Port\n");
        printf("Address: %s\n", _ipAddr.c_str());
        printf("Port: %d\n", _endpointComm.port());
        printf("Exception: %s\n", e.what());
        _connectedCommPort = false;
    }
}

void TrignoEmgClient::SendCommand(int cmdNumber){
    /*
     * This function sends a command through the comm port.  The argument cmdNumber
     * specifies which command to send.  All commands are listed in the std::map
     * variable named _cmds
     */
	std::string cmd = _cmds[cmdNumber];
	if (cmd.compare("") != 0){	// command found
		printf("Command: %s\n", RemoveNewlines(cmd).c_str());
    	try{
          _sockComm.send(boost::asio::buffer(cmd.c_str(), strlen(cmd.c_str())));
          this->GetReplyComm();
        }
        catch (std::exception & e){
          printf("Exception Occured During Write: %s\n", e.what());
        }
	}
	else{	                 // command not found
		printf("Command number %d does not correspond to a command in the command list\n", cmdNumber);
	}
}

bool TrignoEmgClient::IsCommPortConnected(){
    return _connectedCommPort;
}

bool TrignoEmgClient::IsDataPortConnected(){
    return _connectedDataPort;
}

/* Private Functions */
void TrignoEmgClient::GetReplyComm(){
    /*
     * This function receives the incoming messages from the comm port and
     * prints them.
     */

    /* Reset reply string and get new reply */
    memset(_replyComm, 0, sizeof(_replyComm));
    try{
        boost::system::error_code ec;
        size_t n = boost::asio::read(_sockComm,
                                  boost::asio::buffer(_replyComm),
                                  boost::asio::transfer_at_least(1),
                                  ec);

        if (ec){    // error
            printf("Error occured while receiving comm reply from server\n");
        }
        else{       // no error
            /* Convert to std::string, remove newlines */
            std::string temp(_replyComm);
            temp = RemoveNewlines(temp);

            /* Print to command line */
            printf("%s\n", temp.c_str());
        }
    }
    catch (std::exception & e){
        printf("Exception Occured During Comm Read: %s\n", e.what());
    }
}

std::string TrignoEmgClient::RemoveNewlines(std::string str_in){
    /*
     * Trigno commands and responses are sent with a \r\n\r\n at the end of the
     * message.  These are annoying to print so this function removes those and
     * returns a string without the newlines, which is then typically printed
     */
    std::string temp;
    size_t newlineStart = str_in.find("\r\n\r\n");
    if (newlineStart != std::string::npos){
        temp = str_in.replace(newlineStart, 4, "");
    }
    else{
        temp = str_in;
    }
    return temp;
}


void TrignoEmgClient::ReceiveDataStream(){
    /*
     * This function checks if the data port is connected.  If it is it will continously
     * receive data until StopReceiveDataStream() is run or an error while receiving data occurs.
     * It is highly recommended, if not almost completely mandatory, that this
     * function is run in it's own thread. To do this, use code of the form:
     *
     *          std::thread t(&TrignoEmgClient::ReceiveDataStream, &clientVar)
     *
     * Don't forget to rejoin the thread at the end of the script! code: t.join();
     */
    std::array<float,16> temp;
    int bytesAvailable = 0;
    int samplesAvailable = 0;
    _rowCount = 0;
    int rowsFreeInBuff = chunkRows - _rowCount;
    int readBytes = 0;
    while (_connectedDataPort){
        /* Receive 16 sensors * 4 bytes/sensor worth of data and handle it */
        memset(_replyData, 0, sizeof(_replyData));
        boost::system::error_code ec;
        bytesAvailable = (int) _sockData.available();
        samplesAvailable = (int) (bytesAvailable/(_nSensors*_nBytesPerFloat));
        if (samplesAvailable > rowsFreeInBuff){
            samplesAvailable = rowsFreeInBuff;
        }
        readBytes = samplesAvailable*(_nSensors*_nBytesPerFloat);
        if (readBytes > MAXDATALENGTH){
            readBytes = MAXDATALENGTH;
        }

        size_t n = boost::asio::read(_sockData,
                                     boost::asio::buffer(_replyData),
                                     boost::asio::transfer_exactly(readBytes),
                                     ec);
        if (ec){    // error
            printf("Error occured while reading\n");
            _connectedDataPort = false; // will exit loop
        }
        else{       // no error
            /* Save to data queue if specified by flag */
            if (_writeFlag){
                memcpy(&(_dataArr[_rowCount][0]), _replyData, n);
                _rowCount += samplesAvailable;
                if (_rowCount == chunkRows){
                    WriteH5Chunk();
                    _rowCount = 0;
                }
                rowsFreeInBuff = chunkRows - _rowCount;
            }
            else if (_writeToFileStreamFlag){
              memcpy(_dataArrFileStream, _replyData, n);
              for (int iSample=0; iSample<samplesAvailable; iSample++){
                _ofs << _dataArrFileStream[iSample][0] << ","
                    << _dataArrFileStream[iSample][1] << ","
                    << _dataArrFileStream[iSample][2] << ","
                    << _dataArrFileStream[iSample][3] << ","
                    << _dataArrFileStream[iSample][4] << ","
                    << _dataArrFileStream[iSample][5] << ","
                    << _dataArrFileStream[iSample][6] << ","
                    << _dataArrFileStream[iSample][7] << ","
                    << _dataArrFileStream[iSample][8] << ","
                    << _dataArrFileStream[iSample][9] << ","
                    << _dataArrFileStream[iSample][10] << ","
                    << _dataArrFileStream[iSample][11] << ","
                    << _dataArrFileStream[iSample][12] << ","
                    << _dataArrFileStream[iSample][13] << ","
                    << _dataArrFileStream[iSample][14] << ","
                    << _dataArrFileStream[iSample][15] << "\n";
              }
            }

        }
    }
}

void TrignoEmgClient::StartWriting(H5Location * h5loc){
    /*
     * This function starts the writing process.  It is assumed that ReceiveDataStream()
     * has been run and data is being continously received.  Once the _dataArr variable
     * is full, WriteH5Chunk() will be run.  The argument h5loc is a pointer to an
     * h5object where the dataemg will be made. This can either be a Group or File obj.
     */

     /* Check if already writing */
     if (_writeFlag){
         printf("EMG is already being written.  Current write must be stopped before new write can start.\n");
         return;
     }

     /* file space */
     fspaceOffset[0]    = 0;
     fspaceOffset[1]    = 0;
     fspaceDims[0]      = 1;
     fspaceDims[1]      = _nActiveEmgSensors;
     fspaceMaxDims[0]   = H5S_UNLIMITED;
     fspaceMaxDims[1]   = _nActiveEmgSensors;
     fspace = DataSpace(rank, fspaceDims, fspaceMaxDims);

     /* write space */
     writespaceDims[1]  = _nActiveEmgSensors;

     /* Set chunk size in dataset creation property list */
     chunkDims[0]       = chunkRows;
     chunkDims[1]       = _nActiveEmgSensors;
     dsPropList.setChunk(rank, chunkDims);       // Allows dataset to be chunked

    /* Create EMG dataset */
    std::string datasetNameBase = "EMG";
    int datasetNum = 1;
    std::string datasetName = datasetNameBase;
    std::string datasetNameUnsuc = datasetName;
    if (h5loc->exists(datasetNameUnsuc)){
        while (h5loc->exists(datasetNameUnsuc)){
            datasetNameUnsuc = datasetNameBase + std::string("_unsuccessful_") + std::to_string(datasetNum);
            datasetNum++;
        }
        /* renames old EMG dataset as EMG_unsuccessful_# */
        h5loc->move(datasetName, datasetNameUnsuc);
    }
    datasetEmg = h5loc->createDataSet(datasetName,
                                      PredType::NATIVE_FLOAT,
                                      fspace,
                                      dsPropList);

    /* Reset data queues, counters, and flags */
    memset(_dataArr, 0, sizeof(float)*chunkRows*_nSensors);
    _firstWrite = true;
    _writeFlag = true;
    _rowCount = 0;
}

void TrignoEmgClient::StopWriting(){
    /*
     * This function will write whatever values in the _dataArr buffer
     * that haven't yet been written and stop the writing process.
     */
    _writeFlag = false;
    std::thread t(&TrignoEmgClient::WriteH5Chunk, this);
    t.detach();
}

void TrignoEmgClient::WriteH5Chunk(){
    /*
     * The function writes whatever data in the _dataArr buffer that hasn't been
     * written to the emg dataset.
     */
    /* Calculate dataset dimensions and offset for writing new chunk */
    if (_firstWrite){
        fspaceDims[0]       = (hsize_t) _rowCount;
        fspaceOffset[0]     = 0;
        _firstWrite         = false;
    }
    else{
        fspaceDims[0]       += (hsize_t) _rowCount;   // add necessary rows
        fspaceOffset[0]     += (hsize_t) chunkRows;
    }

    /* extend dataset */
    datasetEmg.extend(fspaceDims);

    /* specify where in the file dataspace to place new samples */
    writespaceDims[0] = fspaceDims[0] - fspaceOffset[0];
    fspace = datasetEmg.getSpace();
    fspace.selectHyperslab(H5S_SELECT_SET, writespaceDims, fspaceOffset);

    /* specify where in the memory dataspace to get the new samples */
    mspace.selectNone();
    if (_nActiveEmgSensors != 16){
      hsize_t colDims[rank]   = {(hsize_t) _rowCount, 1};
      hsize_t offset[rank]    = {0, 0};
      /* select column by column */
      for (int i = 0; i<_nActiveEmgSensors; i++){
          offset[1] = _emgList[i] - 1;
          mspace.selectHyperslab(H5S_SELECT_OR, colDims, offset);
      }
    }
    else{
      hsize_t colDims[rank]   = {(hsize_t) _rowCount, 1};
      hsize_t offset[rank]    = {0, 0};
      /* select column by column */
      for (int i = 0; i<_nActiveEmgSensors; i++){
          offset[1] = i;
          mspace.selectHyperslab(H5S_SELECT_OR, colDims, offset);
      }
    }

    /* write to file */
    datasetEmg.write(_dataArr, PredType::NATIVE_FLOAT, mspace, fspace);
}

void TrignoEmgClient::StopReceiveDataStream(){
    _connectedDataPort = false;
}

bool TrignoEmgClient::IsWriting(){
    return _writeFlag;
}

void TrignoEmgClient::StartWritingFileStream(path filepath){
  /* deconstruct filepath into path, filename, extension */
	path pp = filepath.parent_path();
	path fname_stem = filepath.stem();
	path fname_ext = filepath.extension();

	/* Make a path to rename old file with same path, and rename if necessary */
	path p_unsuc = path(filepath.string());
	int unsuc_count = 1;
	std::string fname_unsuc;
	if (is_regular_file(p_unsuc)){
		while (is_regular_file(p_unsuc)){
			fname_unsuc = fname_stem.string() + std::string("_unsuccessful_") + std::to_string(unsuc_count) + fname_ext.string();
			p_unsuc = path(pp.string()) /= path(fname_unsuc);
			unsuc_count++;
		}
		rename(filepath, p_unsuc);
	}

  /* Start file stream */
    _ofs.close();
    _ofs.open(filepath);
    _writeToFileStreamFlag = true;
}

void TrignoEmgClient::StopWritingFileStream(){
  _writeToFileStreamFlag = false;
}

void TrignoEmgClient::IsSensorPaired(int sensorNumber){
    char cmdStr[50];
    char cmdStrNoNewline[50];
    sprintf(cmdStr, "SENSOR %d PAIRED?\r\n\r\n", sensorNumber);
    sprintf(cmdStrNoNewline, "SENSOR %d PAIRED? ", sensorNumber);
    printf("Command: %s", cmdStrNoNewline);
    try{
      _sockComm.send(boost::asio::buffer(cmdStr, strlen(cmdStr)));
      this->GetReplyComm();
    }
    catch (std::exception & e){
      printf("Exception Occured During Write: %s\n", e.what());
    }
}
