#include <string>
#include "H5Cpp.h"

using namespace H5;


H5File * CreateOrOpenH5File(const std::string & filename){
    Exception::dontPrint();
    H5File * file = 0;

    try{
        /* Open file if it exists */
        file = new H5File(filename.c_str(), H5F_ACC_RDWR);
    }
    catch(const FileIException&){
        /* Create file if it does not exist */
        file = new H5File(filename.c_str(), H5F_ACC_TRUNC);
    }

    return file;
}

Group * CreateOrOpenGroup(H5Location * h5loc, H5std_string & group_name){

    Exception::dontPrint();
    Group * group = 0;

    try{
        /* Open group if it exists */
        group = new Group(h5loc->openGroup(group_name));
    } catch(Exception & e){
        group = new Group(h5loc->createGroup(group_name));
    }

    return group;
}



void CreateStringAttribute(H5Object * h5obj, H5std_string attr_name, H5std_string attr_val){
    if (!(h5obj->attrExists(attr_name))){
        // Create new dataspace for attribute -- H5S_SCALAR means the dataspace has 1 element
        DataSpace ds = DataSpace(H5S_SCALAR);

        // Create new string datatype for attribute
        StrType strdatatype(PredType::C_S1, 256); // of length 256 characters

        // Create attribute and write to it
        Attribute attr = h5obj->createAttribute(attr_name, strdatatype, ds);
        attr.write(strdatatype, attr_val);
    }
}

void CreateNumericAttribute(H5Object * h5obj, H5std_string attr_name, double attr_val){
    if (!(h5obj->attrExists(attr_name))){
        // Create new dataspace for attribute -- H5S_SCALAR means the dataspace has 1 element
        DataSpace ds = DataSpace(H5S_SCALAR);

        // Create float datatype
        FloatType floatdatatype(PredType::NATIVE_DOUBLE);

        // Create attribute and write to it
        Attribute attr = h5obj->createAttribute(attr_name, floatdatatype, ds);
        attr.write(floatdatatype, &attr_val);
    }
}


// DataSet InitDataSet2D(H5Location & h5loc, std::string dsetName, hsize_t nCols, const DataType & data_type, hsize_t chunkRows = 10000){
//     /* Init DataSpace */
//     int rank = 2;
//     hsize_t dims[rank] = {0, nCols};
//     hsize_t dimsMax[rank] = {H5S_UNLIMITED, nCols};
//     DataSpace dspace(rank, dims, dimsMax);
//
//     /* Init Dataset Property List */
//     DSetCreatPropList dsPropList;
//     hsize_t chunkDims[rank] = {chunkRows, nCols};
//     dsPropList.setChunk(rank, chunkDims);
//
//     /* Create DataSet */
//     DataSet dset = h5loc.createDataSet(dsetName, data_type, dspace, dsPropList);
//     return dset;
// }
//
// template <typename T>
// void AppendToDataSet2D(DataSet & dset, T * buff, hsize_t nRowsToAppend, int nCols, const DataType & data_type){
//     /* copy buff in case value pointed to by buff changes */
//     T temp[(int)nRows][nCols];
//     memcpy(temp, buff, sizeof(T)*(int)nRows*nCols);
//
//     /* Get current dimensions, will be used to calculate offset */
//     DataSpace fspace = dset.getSpace();
//     int rank = 2;
//     hsize_t dimsOld[rank];
//     rank = fspace.getSimpleExtentDims(dimsOld);
//
//     /* Calculate new dataspace dimensions */
//     hsize_t offset[rank] = {dimsOld[0], 0};
//     hsize_t dimsNew[rank] = {dimsOld[0]+nRowsToAppend, dimsOld[1]};
//     hsize_t dimsWrite[rank] = {nRowsToAppend, dimsOld[1]};
//
//     /* Extend dataset dimensions */
//     dset.extend(dimsNew);
//
//     /* Specify where in file space the new data will be written */
//     fspace = dset.getSpace();
//     fspace.selectHyperslab(H5S_SELECT_SET, dimsWrite, offset);
//
//     /* Specify memory space */
//     DataSpace mspace(rank, dimsWrite);
//     mspace.selectAll();
//
//     /* Write To DataSet */
//     dset.write(temp, data_type, mspace, fspace);
// }
