/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifdef QRB5165

#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>  // strerror
#include <algorithm> // find
#include <unistd.h>
#include <thread>
extern "C" {
#include <sys/stat.h>
}

#include <sys/resource.h>
#include <sys/syscall.h>
#include <iostream>
#include <sys/ioctl.h>

// ModalAI libs
#include <modal_journal.h>
#include <cci_direct.h>

// local includes
#include "tof_interface.hpp"

// royale includes
#include "common/MakeUnique.hpp"
#include "config/CoreConfigAdapter.hpp"

using namespace platform;
using namespace royale;
using namespace royale::hal;
using namespace royale::pal;

std::vector <uint32_t> mShortRangeFramerates;
std::vector <uint32_t> mLongRangeFramerates;
std::vector <uint32_t> mExtraLongRangeFramerates;


// -----------------------------------------------------------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------------------------------------------------------
static const RoyaleListenerType RoyaleListenerTypes[] = {LISTENER_DEPTH_DATA,
                                                         LISTENER_SPARSE_POINT_CLOUD,
                                                         LISTENER_DEPTH_IMAGE,
                                                         LISTENER_IR_IMAGE};


//----------------------------------------------------------------------------------------
// CRC tab for validation
//----------------------------------------------------------------------------------------
static uint32_t crc32Tab[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};


// -----------------------------------------------------------------------------------------------------------------------------
// BridgeImager class implementation
// -----------------------------------------------------------------------------------------------------------------------------
BridgeImager::BridgeImager(std::shared_ptr<I2cAccess> i2cAccess) {
    m_i2cAccess = i2cAccess;
}

int BridgeImager::setupEeprom() {
    getEepromHeader();
    calStringCreate();
    
    if (!calFileExist()) {
        calEepromRead();
        calEepromDumpToFile();

        if (calDataParse()) {
            calFileDump();
        }
    }

    return 0;
}

void BridgeImager::getEepromHeader() {
    const int HEADER_SIZE = 67;
    std::vector<uint8_t> eepromHeader;

    // read header data
    eepromHeader.resize(HEADER_SIZE);
    m_i2cAccess->readI2cSeq(EEPROM_1ST_PAGE_ADDR << 1, 0, I2cAddressMode::I2C_16BIT, eepromHeader, TOF_I2C_DATA_TYPE_BYTE);

    // Copy data to eeprom struct
    calEepromHeader.insert(calEepromHeader.end(), eepromHeader.begin(), eepromHeader.end());
}

void BridgeImager::calStringCreate() {
    // Get header to extract serial number
    calDataHeaderv7_t header;
    std::copy(calEepromHeader.begin(), calEepromHeader.begin()+sizeof(header), header.data);

    const std::string camPath = "/data/misc/camera/";

    // Create cal file paths
    calFilePath              = camPath + std::string(header.serial_number, 19) + "/";
    calEepromFileNamePrivate = calFilePath + "pmd.spc";
    calEepromFileNameDump    = calFilePath + "tof_cal_eeprom.bin";
    calLensParams            = calFilePath + "irs10x0c_lens.cal";
}

// dump royale lense params
void BridgeImager::dumpLensParameters (std::pair<float, float> principalPoint, std::pair<float, float> focalLength,
                        std::pair<float, float> distortionTangential, std::vector<float> &  distortionRadial) {

    const char *lensParamsFilePath = calLensParams.c_str();

    std::ofstream ofs;
    ofs.open(lensParamsFilePath, std::ofstream::out | std::ofstream::trunc);
    if (ofs.is_open()) {
        ofs << "VERSION:1.0:VERSION" << std::endl;
        ofs << "#px,py,cx,cy,tan-coeff[0..1], rad-coeff[0..n-1]" << std::endl;
        ofs << principalPoint.first << " " << principalPoint.second << " "
            << focalLength.first << " " << focalLength.second << " "
            << distortionTangential.first << " "
            << distortionTangential.second;
        for (int i = 0; i < int(distortionRadial.size()); i++ ) {
           ofs << " " << distortionRadial.at(i);
        }
        ofs << std::endl;

        ofs.close();
    }
    else {
        M_ERROR("Cannot dump ToF sensor lens params to %s\n", lensParamsFilePath);
    }
}


// Single CCI read
void BridgeImager::readImagerRegister(uint16_t regAddr, uint16_t &value) {
    std::vector<uint8_t> buffer;
    m_i2cAccess->readI2c(BridgeImager::imagerSlave,
                        I2cAddressMode::I2C_16BIT,
                        regAddr,
                        buffer);

    value = (((buffer[0] << 8) & 0xff00) | buffer[1]);
}

// Single CCI write
void BridgeImager::writeImagerRegister(uint16_t regAddr, uint16_t value) {
    std::vector<uint8_t> buffer;
    buffer.resize(2);

    // \todo take care about HTOL conversion here!
    std::memcpy(buffer.data(), &value, 2);

    m_i2cAccess->writeI2c(BridgeImager::imagerSlave, I2cAddressMode::I2C_16BIT, regAddr, buffer);
}

// Burst CCI direct read
void BridgeImager::readImagerBurst(uint16_t firstRegAddr, std::vector<uint16_t> &values) {
    uint16_t addr = firstRegAddr;

    for (uint32_t k = 0; k < values.size(); k++) {
        readImagerRegister(addr, values[k]);
        addr++;
    }
}

// Burst CCI direct write
void BridgeImager::writeImagerBurst(uint16_t firstRegAddr, const std::vector<uint16_t> &values) {
    uint16_t addr;
    size_t i, size = values.size();
    std::map<uint16_t, uint16_t> data;

    for (i=0,addr=firstRegAddr; i<size; i++,addr++) {
        std::pair<std::map<uint16_t,uint16_t>::iterator,bool> ret;
        ret = data.insert(std::pair<uint16_t,uint16_t>(addr, values[i]));
    }

    m_i2cAccess->writeI2cArray(BridgeImager::imagerSlave, I2cAddressMode::I2C_16BIT, data);

}

// used by royale
void BridgeImager::sleepFor(std::chrono::microseconds sleepDuration) {
    std::this_thread::sleep_for(sleepDuration);
}

// Check for preexisting cal files
bool BridgeImager::calFileExist() {
    struct stat buf;

    // check if file with "Private" calibration data is present
    const char *name = calEepromFileNamePrivate.c_str();
    bool exist = (stat(name, &buf) == 0);

    return exist;
}

// Parse header data by format, V7 or older
bool BridgeImager::calDataParse() {
    // Check header version
    int16_t headerVersion = 0;
    if (!getEepromHeaderVersion(headerVersion)) {
        M_ERROR("Failed to get EEPROM header version\n");
        return false;
    }

    // Parse and validate header data
    if (headerVersion == 0x07) { // V7 header validation method
        bool calIsValid = calDataValidatev7(calEepromData);
        if (!calIsValid) {
            M_ERROR("Calibration data is not valid");
            return false;
        }

        calDataHeaderv7_t headerv7;
        std::copy(calEepromData.begin(), calEepromData.begin()+16, headerv7.data);
        auto unknownBegin = calEepromData.begin() + sizeof(headerv7);
        auto unknownEnd = unknownBegin + headerv7.data_size;
        calDataUnknown.insert(calDataUnknown.end(), unknownBegin, unknownEnd);

        calDataHeaderv7_t header;
        std::copy(calEepromData.begin(), calEepromData.begin()+67, header.data);
        unknownBegin = calEepromData.begin() + sizeof(header);
        unknownEnd = unknownBegin + header.data_size;
        calDataUnknown.insert(calDataUnknown.end(), unknownBegin, unknownEnd);
    }
    else { // not V7 use old header validation method
        M_ERROR("Invalid header version\n");
        return false;
    }

    return true;
}

// I2C seq read to extract all EEPROM data
void BridgeImager::calEepromRead() {
    std::vector<uint8_t> page;

    // iterate through all EEPROM pages
    for (uint8_t i = 0; i < EEPROM_PAGE_NUM; i++) {
        page.clear();
        page.resize(EEPROM_PAGE_SIZE);

        // read single page
        m_i2cAccess->readI2cSeq((EEPROM_1ST_PAGE_ADDR+i) << 1, 0, I2cAddressMode::I2C_16BIT, page, TOF_I2C_DATA_TYPE_BYTE);

        // collect all data in single array
        calEepromData.insert(calEepromData.end(), page.begin(), page.end());
    }
}

// Obtain header version
bool BridgeImager::getEepromHeaderVersion(int16_t& version) {

    calDataHeaderv7_t header;

    // check for original header, since the version info is in the same location for v7
    std::copy(calEepromData.begin(), calEepromData.begin() + sizeof(header), header.data);

    // check Magic string, same location as v7
    std::string magic="PMDTEC";
    size_t len = magic.size();

    if ( magic.compare(0, len, header.magic, len) == 0 ) {
        version = header.version;
        return true;
    }

    return false;
}

// dump parsed data from EEPROM file
void BridgeImager::calFileDump() {
    // dump "Private" calibration file
    const char *name = calEepromFileNamePrivate.c_str();
    std::ofstream f (name, std::ios_base::out | std::ios_base::binary);
    if (!f.is_open()) {
        M_ERROR("Failed to open file \"%s\"", name);
        return;
    }

    f.write((char *)&calDataUnknown[0], calDataUnknown.size());
    f.close();
}

// Dump EEPROM data read directly from sensor
void BridgeImager::calEepromDumpToFile() {
    int ret;
    struct stat buf;

    const char *folderName = calFilePath.c_str();
    const char *eepromCalName = calEepromFileNameDump.c_str();

    if ( !(stat(folderName, &buf) == 0) ) {
        // directory doesn't exist, create it
        ret = mkdir(folderName, 0777);
        if (ret < 0) {
            M_ERROR("Failed to create directory \"%s\"", folderName);
            return;
        }
    }

    if ( !(stat(eepromCalName, &buf) == 0) ) {
        std::ofstream f (eepromCalName, std::ios_base::out | std::ios_base::binary);
        if (!f.is_open()) {
            M_ERROR("Failed to open file \"%s\"", eepromCalName);
            return;
        }

        f.write((char *)&calEepromData[0], calEepromData.size());
        f.close();
    }
}

// validates V7 header
bool BridgeImager::calDataValidatev7(std::vector<uint8_t> &data) {
    calDataHeaderv7_t header;
    std::copy(data.begin(), data.begin()+sizeof(header), header.data);

    // check Magic string
    std::string magic="PMDTEC";
    size_t len = magic.size();
    bool rc = (magic.compare(0, len, header.magic, len) == 0);

    // check CRC
    uint8_t *block = data.data() + sizeof(header);
    uint32_t crc = crc32(0, block, header.data_size);
    rc &= (crc == header.data_crc32);

    return rc;
}

// Utility function for validating ToF EEPROM data
uint32_t BridgeImager::crc32(uint32_t crc, const uint8_t *buf, size_t size) {
    const uint8_t *p;

    p = buf;
    crc = crc ^ ~0U;

    while (size--)
        crc = crc32Tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);

    return crc ^ ~0U;
}

// -----------------------------------------------------------------------------
// I2CAccess class implemenatation
// -----------------------------------------------------------------------------

// CCI direct sequence read
void I2cAccess::readI2cSeq(uint8_t devAddr, uint16_t regAddr, I2cAddressMode addrMode, std::vector<uint8_t> &data, I2C_DATA_TYPE dataType) {
    int ret;

    if (addrMode == I2cAddressMode::I2C_NO_ADDRESS){
        // TODO: figure out no address mode
        // M_ERROR("I2C no address mode is not supported must be 8 or 16 bit\n");
        return;
    }
    cci_data_type_t cciDataType = (dataType == TOF_I2C_DATA_TYPE_BYTE) ? CCI_8BIT : CCI_16BIT;
    cci_data_type_t cciAddrType = (addrMode == I2cAddressMode::I2C_8BIT) ? CCI_8BIT : CCI_16BIT;

    for (uint32_t i = regAddr;i<data.size();i++) {
        regAddr = i;
        ret = voxl_cci_read(m_cameraId, devAddr, regAddr, cciAddrType, &data[i], cciDataType);
        if (ret < 0) {
            M_ERROR("Failed CCI read\n");
        }
    }
}

// Single CCI direct read
void I2cAccess::readI2c(uint8_t devAddr, I2cAddressMode addrMode, uint16_t regAddr, std::vector<uint8_t> &buffer) {
    int ret;

    // At most we'll be reading a word of data, so resize to 2
    buffer.clear();
    buffer.resize(2);

    if (addrMode == I2cAddressMode::I2C_NO_ADDRESS){
        // TODO: figure out no address mode
        // M_ERROR("I2C no address mode is not supported must be 8 or 16 bit\n");
        return;
    }
    cci_data_type_t cciAddrType = (addrMode == I2cAddressMode::I2C_8BIT) ? CCI_8BIT : CCI_16BIT;

    // ToF data read through this function will always be 16-Bit data
    ret = voxl_cci_read(m_cameraId, devAddr, regAddr, cciAddrType, &buffer[0], CCI_16BIT);
    if (ret < 0) {
        M_ERROR("Failed CCI read\n");
    }
}

// Single CCI direct write
void I2cAccess::writeI2c(uint8_t devAddr, I2cAddressMode addrMode, uint16_t regAddr, const std::vector<uint8_t> &buffer) {
    int ret;

    if (addrMode == I2cAddressMode::I2C_NO_ADDRESS){
        // TODO: figure out no address mode
        // M_ERROR("I2C no address mode is not supported must be 8 or 16 bit\n");
        return;
    }
    cci_data_type_t cciAddrType = (addrMode == I2cAddressMode::I2C_8BIT) ? CCI_8BIT : CCI_16BIT;

    // ToF data written through this function will always be 16-Bit data
    ret = voxl_cci_write(m_cameraId, devAddr, regAddr, cciAddrType, (uint8_t *)&buffer[0], CCI_16BIT);
    if (ret < 0) {
        M_ERROR("Failed CCI read\n");
    }
}

// CCI direct array write
void I2cAccess::writeI2cArray(uint8_t devAddr, I2cAddressMode addrMode, const std::map <uint16_t, uint16_t> &reg_map) {
    int ret;

    if (addrMode == I2cAddressMode::I2C_NO_ADDRESS){
        // TODO: figure out no address mode
        // M_ERROR("I2C no address mode is not supported must be 8 or 16 bit\n");
        return;
    }
    cci_data_type_t cciAddrType = (addrMode == I2cAddressMode::I2C_8BIT) ? CCI_8BIT : CCI_16BIT;

    size_t reg_array_size = reg_map.size();
    uint16_t regAddr[reg_array_size] = {0};
    uint16_t regData[reg_array_size] = {0};

    // convert map into seperate arrays for voxl_cci_write_word_array
    uint32_t idx;
    std::map<uint16_t, uint16_t>::const_iterator iter;
    for (iter=reg_map.begin(), idx=0; iter!=reg_map.end(); iter++, idx++) {
        regAddr[idx] = iter->first;
        regData[idx] = iter->second;
    }

    // Writes word of data
    ret = voxl_cci_write_word_array(m_cameraId, devAddr, regAddr, cciAddrType, regData, reg_array_size);
    if (ret < 0) {
        M_ERROR("Failed CCI read\n");
    }

}

// Used for resetting sensor via GPIO, currently unsupported
int I2cAccess::setGPIO(uint16_t gpio, uint16_t data) {
    M_ERROR("Setting GPIO is currently unsupported\n");
    return -1;
}


// -----------------------------------------------------------------------------------------------------------------------------
// BridgeDataReceiver class implementation
// -----------------------------------------------------------------------------------------------------------------------------
void BridgeDataReceiver::setBufferCaptureListener (IBufferCaptureListener *collector) {
    std::lock_guard<std::mutex> guard (m_changeListenerLock);
    // only save the pointer to the provided collector
    // the framecollector is used for calling the callback
    m_bufferCaptureListener = collector;
}

std::size_t BridgeDataReceiver::executeUseCase (int width, int height, std::size_t preferredBufferCount) {
    // set the MIPI buffer sizes to the correct new buffer size
    // if we can only work with pre-defined buffer sizes, the interface can also change
    // this would be the most flexible approach of course
    //
    // returns the number of buffers which are successfully allocated
    return preferredBufferCount; // has all the buffers we requested
}

void BridgeDataReceiver::startCapture() {
    // empty implementation, nothing required
}

void BridgeDataReceiver::stopCapture() {
    // empty implementation, nothing required
}

float BridgeDataReceiver::getPeakTransferSpeed() {
    return 10.0f;
}

void BridgeDataReceiver::queueBuffer (ICapturedBuffer *buffer) {
    /*
    Vector<royale::hal::ICapturedBuffer *> &royaleBufs = m_hwi->getRoyaleDequeuedBufs();
    royaleBufs.add(buffer);
    */

    delete buffer;
}

void BridgeDataReceiver::dataCallback(uint16_t * pixelData, uint64_t ts) {
    std::lock_guard<std::mutex> guard (m_changeListenerLock);

    CapturedBuffer *buffer = new CapturedBuffer (pixelData, ts);
    //CapturedBuffer *buffer = new CapturedBuffer (data);
    if (m_bufferCaptureListener != nullptr) {
        m_bufferCaptureListener->bufferCallback (buffer);
    }
    else {
        queueBuffer (buffer);
    }
}

bool BridgeDataReceiver::isConnected() const {
    return true;
}

royale::Vector<royale::Pair<royale::String, royale::String>> BridgeDataReceiver::getBridgeInfo() {
    return royale::Vector<royale::Pair<royale::String, royale::String>>();
}

void BridgeDataReceiver::setEventListener (royale::IEventListener *listener) {
    // empty implementation
}


// -----------------------------------------------------------------------------------------------------------------------------
// TOFBridge friend classes implementations // TODO: validate
// -----------------------------------------------------------------------------------------------------------------------------

status_t TOFBridge::onRoyaleDepthData(const void *data, uint32_t size, int64_t timestamp, RoyaleListenerType dataType) {
    if (!mDepthChannel) {
        M_WARN("%s No DepthChannel provided!", __func__);
        return -1;
    }

    return mDepthChannel->RoyaleDataDone(data, size, timestamp, dataType) ? 0 : -1;
}

// IR image callback
void TOFBridge::IRImageListener::onNewData (const royale::IRImage *data) {
    if (mTOFBridge != NULL && mTOFBridge->mDepthChannel != NULL) {
        mTOFBridge->onRoyaleDepthData(static_cast<const void*> (data), 0, data->timestamp, LISTENER_IR_IMAGE);
    }
}

// Depth image callback
void TOFBridge::DepthImageListener::onNewData (const royale::DepthImage *data) {
    if (mTOFBridge != NULL && mTOFBridge->mDepthChannel != NULL) {
        mTOFBridge->onRoyaleDepthData(static_cast<const void *>(data), 0, data->timestamp, LISTENER_DEPTH_IMAGE);
    }
}

// Sparse Pointcloud callback
void TOFBridge::SparsePointCloudListener::onNewData (const royale::SparsePointCloud *data) {
    if (mTOFBridge != NULL && mTOFBridge->mDepthChannel != NULL) {
        mTOFBridge->onRoyaleDepthData(static_cast<const void*> (data), 0, data->timestamp, LISTENER_SPARSE_POINT_CLOUD);
    }
}

// Depth data callback
void TOFBridge::DepthDataListener::onNewData (const royale::DepthData *data) {
    if (mTOFBridge != NULL && mTOFBridge->mDepthChannel != NULL) {
        mTOFBridge->onRoyaleDepthData(static_cast<const void*> (data), 0, data->timeStamp.count(), LISTENER_DEPTH_DATA);
    }
}


// -----------------------------------------------------------------------------------------------------------------------------
// TOFBridge class implementation
// -----------------------------------------------------------------------------------------------------------------------------

TOFBridge::TOFBridge() {
    moduleConfig = getModuleConfigCustom();

    listeners.irImage = nullptr;
    listeners.depthImage = nullptr;
    listeners.sparsePointCloud = nullptr;
    listeners.depthData = nullptr;
    mFrameRate = 10;
    mDistanceRange = LONG_RANGE;  // SHORT_RANGE (phase 5) is not currently supported by camera pipe line
    if (mLongRangeFramerates.size() > 0)
    {
        mFrameRate = mLongRangeFramerates[0];
    }
    mExposureMode =  royale::ExposureMode::AUTOMATIC;
    mExposureTime = 200; //us
    paramChange = 0;
}

TOFBridge::~TOFBridge() {
    i2cAccess.reset();
    bridgeImager.reset();
    bridgeReceiver.reset();
    royaleCamera.reset();
    delete listeners.irImage;
    delete listeners.depthImage;
    delete listeners.sparsePointCloud;
    delete listeners.depthData;
}

bool TOFBridge::isTOFCam(int32_t width, int32_t height) {
    if (width == TOF_1PHASE_WIDTH && height == TOF_1PHASE_HEIGHT)
        return true;
    if (width == TOF_5PHASE_WIDTH && height == TOF_5PHASE_HEIGHT)
        return true;
    if (width == TOF_9PHASE_WIDTH && height == TOF_9PHASE_HEIGHT)
        return true;
    if (width == TOF_11PHASE_WIDTH && height == TOF_11PHASE_HEIGHT)
        return true;
    return false;
}

void TOFBridge::setChange(RoyaleParamChange param) {
    std::lock_guard<std::mutex> guard (paramChangeLock);
    uint8_t mask = static_cast<RoyaleParamChange>(param);
    paramChange |= mask;
}

std::pair<uint32_t, uint32_t> TOFBridge::getExposureLimits() {
    std::pair <uint32_t, uint32_t> expLimits;
    expLimits = std::make_pair(mExposureLimits.first, mExposureLimits.second);
    M_DEBUG("%s@%d: X(expLimits= (.min= %d, .max= %d))\n", __PRETTY_FUNCTION__, __LINE__, expLimits.first, expLimits.second);
    return expLimits;
}

royale::usecase::UseCaseDefinition *TOFBridge::getUseCaseDef (royale::String useCaseName) {

    using namespace royale;
    using namespace royale::config;
    std::unique_ptr <CoreConfig> configData = common::makeUnique<CoreConfig> (moduleConfig->coreConfigData);
    std::shared_ptr <CoreConfigAdapter> configAdapter = std::make_shared<CoreConfigAdapter> (std::move(configData));
    auto useCaseList = configAdapter->getSupportedUseCases();

    for (royale::usecase::UseCase useCase : useCaseList) {
        if (useCase.getCallbackData() != CallbackData::Depth) {
            continue;
        }
        royale::String name = useCase.getName();
        if (name.find(useCaseName) == std::string::npos) {
            continue;
        }
        royale::usecase::UseCaseDefinition *def = useCase.getDefinition();
        return def;
    }
    return NULL;
}

uint32_t TOFBridge::getExposureTime(royale::String useCaseName) {
    uint32_t expTime = 0;
    royale::usecase::UseCaseDefinition *def = getUseCaseDef(useCaseName);
    if (NULL == def) {
        M_WARN("%s@%d: Failed to get useCase definition",
            __PRETTY_FUNCTION__, __LINE__);
    }

    //const royale::Vector<royale::usecase::ExposureGroup> exposureGroups = def->getExposureGroups();
    const royale::Vector<uint32_t> expTimes = def->getExposureTimes();
    expTime = expTimes[expTimes.size() - 1];

    return expTime;
}

int TOFBridge::setUseCase(RoyaleDistanceRange range, uint8_t frameRate) {
    std::vector <uint32_t> *list = NULL;
    switch (range) {
        case SHORT_RANGE:
            list = &TOFBridge::mShortRangeFramerates;
            break;
        case LONG_RANGE:
            list = &TOFBridge::mLongRangeFramerates;
            break;
        default:
            M_WARN("%s@%d: Unknown range= %d",
                __PRETTY_FUNCTION__, __LINE__, range);
            return -1;
    }

    bool validated = (std::find(list->begin(), list->end(), frameRate) != list->end());
    if (false == validated) {
        M_WARN("%s@%d: Requested usecase= \"%dph@%dfps\" is NOT supported, No change\n",
              __PRETTY_FUNCTION__, __LINE__, range, frameRate);
        return -1;
    }
    royale::String usecase = "MODE_" + std::to_string(range) + '_' + std::to_string(frameRate) +"FPS";
    CameraStatus ret = royaleCamera->setUseCase(usecase);
    if (CameraStatus::SUCCESS != ret) {
        M_ERROR("Setting Royale usecases: %s ret %d!\n", usecase.c_str(), (int)ret);
        return -1;
    }
    // update usecase related params
    mDistanceRange = range;
    setChange(DISTANCE_RANGE);
    mFrameRate = frameRate;
    setChange(FRAME_RATE);
    ret = royaleCamera->getExposureLimits(mExposureLimits);
    setChange(EXPOSURE_LIMITS);
    mExposureTime = getExposureTime(usecase);
    setChange(EXPOSURE_TIME);
    mUseCaseName = usecase;
    M_DEBUG("Changed to usecase %s exposure time limits [ %d - %d ], cur exposure_time %u, exposure mode %d\n",
          usecase.c_str(), mExposureLimits.first, mExposureLimits.second, mExposureTime, mExposureMode);

    return 0;
}

status_t TOFBridge::setup() {
    int interfaceRet;

    // Create I2C interface
    i2cAccess = std::make_shared<I2cAccess>(cameraId);

    // Create Bridge Imager interface
    bridgeImager = std::make_shared<BridgeImager>(i2cAccess);
    interfaceRet = bridgeImager->setupEeprom();
    if (interfaceRet < 0) {
        M_ERROR("Failed to initialize Bridge Imager\n");
        return BAD_VALUE;
    }

    // create an IBridgeDataReceiver implementation in order to receive the data
    bridgeReceiver = std::make_shared<BridgeDataReceiver>();

    platform::CameraFactory factory;
    royaleCamera = factory.createDevice(moduleConfig, bridgeImager, bridgeReceiver, i2cAccess);
    if (royaleCamera == nullptr) {
        M_ERROR("Royale createDevice!");
        return BAD_VALUE;
    }

    CameraStatus ret = royaleCamera->initialize();
    if (CameraStatus::SUCCESS != ret) {
        M_ERROR(" Royale device initialize! ret: 0x%08x(%d)\n", ret,ret);
        return BAD_VALUE;
    }

    LensParameters theLensParams;
    ret = royaleCamera->getLensParameters (theLensParams);
    if (CameraStatus::SUCCESS != ret) {
        M_ERROR("Royale getLensParameters!\n");
        return BAD_VALUE;
    }
    else {
        std::vector<float> radial;
        for (int i = 0; i < int(theLensParams.distortionRadial.size()); i++ ) {
            radial.push_back(theLensParams.distortionRadial.at(i));
        }

        bridgeImager->dumpLensParameters(std::make_pair(theLensParams.principalPoint.first, theLensParams.principalPoint.second),
                                         std::make_pair(theLensParams.focalLength.first, theLensParams.focalLength.second),
                                         std::make_pair(theLensParams.distortionTangential.first, theLensParams.distortionTangential.second),
                                         radial);
    }

    royale::CameraAccessLevel accessLevel;
    ret = royaleCamera->getAccessLevel (accessLevel);
    M_DEBUG("%s accessLevel %d, ret %d\n", __func__, accessLevel, ret);
    setUseCase(mDistanceRange,  mFrameRate);
    ret = royaleCamera->setExposureMode(mExposureMode);

    return NO_ERROR;

}

android::status_t TOFBridge::populateSupportedUseCases(std::vector<uint8_t> &long_range, std::vector<uint8_t> &short_range,
                                                  uint8_t & default_range, uint8_t & default_data_output,
                                                  std::pair<int64_t,int64_t>& default_exp_time_limits,
                                                  uint32_t & default_fps, uint32_t & default_exp_time  ) {
    using namespace royale;
    using namespace royale::config;

    std::shared_ptr<royale::config::ModuleConfig> moduleConfig = platform::getModuleConfigCustom();

    M_VERBOSE("Found Royale module config: imagerType %d, illuminationConfig.dutyCycle: %d temp_sensor_type: %d\n",
          moduleConfig->imagerConfig.imagerType,
          moduleConfig->illuminationConfig.dutyCycle,
          moduleConfig->temperatureSensorConfig.type);

    royale::String modeStr = "MODE";
    std::unique_ptr <CoreConfig> configData = common::makeUnique<CoreConfig> (moduleConfig->coreConfigData);
    std::shared_ptr <CoreConfigAdapter> configAdapter = std::make_shared<CoreConfigAdapter> ( std::move(configData));

    M_VERBOSE("Found Royale module config: maxImgW %d maxImgH %d frameTxMode %d camName %s\n",
          configAdapter->getMaxImageWidth(),
          configAdapter->getMaxImageHeight(),
          configAdapter->getFrameTransmissionMode(),
          configAdapter->getCameraName().c_str());

    M_VERBOSE("Found Royale module config: tempLimitSoft %f tempLimitHard %f autoExpoSupported %s\n",
          configAdapter->getTemperatureLimitSoft(),
          configAdapter->getTemperatureLimitHard(),
          configAdapter->isAutoExposureSupported()? "yes":"no");

    auto useCaseList = configAdapter->getSupportedUseCases();
    royale::Vector<royale::String> useCases;
    default_exp_time =  100; //us

    for (royale::usecase::UseCase usecase : useCaseList) {
        if (usecase.getCallbackData() != CallbackData::Depth) continue;
        royale::String name = usecase.getName();
        useCases.push_back (name);

        if (name.find(modeStr) != std::string::npos) {
            royale::usecase::UseCaseDefinition *def = usecase.getDefinition();
            int phases = def->getRawFrameCount();
            int fps = def->getTargetRate();
            M_VERBOSE("Found Royale usecase: %s - phases: %d fps: %d\n", name.c_str(), phases, fps);

            const royale::Vector<royale::usecase::ExposureGroup> groups = def->getExposureGroups();
            for (size_t i = 0; i < groups.size();i++) {
                royale::usecase::ExposureGroup group = groups[i];
                M_VERBOSE("Found Royale usecase: %s - exposure group[%d] = %s\n",
                      name.c_str(), i, group.m_name.c_str());
            }

            const royale::Vector<royale::Pair<uint32_t, uint32_t>> exp_limit_list = def->getExposureLimits();
            for (size_t i = 0; i < exp_limit_list.size();i++) {
                royale::Pair<uint32_t, uint32_t> each_limit = exp_limit_list[i];
                M_VERBOSE("Found Royale usecase: %s - exp_limit[%d] = (%u %u)\n",
                      name.c_str(), i, each_limit.first, each_limit.second);
            }

            const royale::Vector<uint32_t> expTimes = def->getExposureTimes();
            for (size_t i = 0; i < expTimes.size();i++) {
                M_VERBOSE("Found Royale usecase: %s - exp_time[%d] = %u\n",
                      name.c_str(), i, expTimes[i]);
            }

            if (fps == 10 && phases == LONG_RANGE) {
                default_exp_time_limits = std::make_pair(int64_t(exp_limit_list[0].first), int64_t(exp_limit_list[0].second));//us
                default_exp_time=expTimes[0];
            }

            if (phases == SHORT_RANGE) {
                TOFBridge::mShortRangeFramerates.push_back(fps);
                short_range.push_back( (uint8_t) fps);
            } else if (phases == LONG_RANGE) {
                TOFBridge::mLongRangeFramerates.push_back(fps);
                long_range.push_back( (uint8_t) fps);
            } else if (phases == EXTRA_LONG_RANGE) {
                TOFBridge::mExtraLongRangeFramerates.push_back(fps);
                //extra_long_range.push_back( (uint8_t) fps);
            }
        } else {
            // @todo: These may be EYESAFETY modes, handle later.
            M_VERBOSE("ROYALE: usecase non-mode %s\n", name.c_str());
        }
    }

    moduleConfig.reset();

    default_range = LONG_RANGE;
    default_data_output = LISTENER_SPARSE_POINT_CLOUD;
    default_exp_time_limits = std::make_pair(int64_t(1), int64_t(650));//us
    default_fps = 15;
    if (TOFBridge::mShortRangeFramerates.size() || TOFBridge::mLongRangeFramerates.size())
        return NO_ERROR;

    M_ERROR("Apparently no supported framerates were found for short and long range!");
    return BAD_VALUE;
}

void TOFBridge::setInitDataOutput(RoyaleListenerType _dataOutput) { 
    // this func is supposed to be called after setup(),  but before startCapture()
    if (!royaleCamera) {
        M_ERROR("%s: Royale not initialized. Check if setup() is called first\n", __func__);
        return;
    }

    // Check if a valid listener type... Currently doesn't support bit-stacked values
    // (e.g. can't use more than one listener simulatensouly)
    bool found = false;
    for (const auto lt : RoyaleListenerTypes) {
        if (lt == _dataOutput) {
            found = true;
            dataOutput = _dataOutput;
        }
    }

    if (!found) {
        M_ERROR("%s invalid data Output for Royale: %d/0x%x, use default 0x%x\n",
        __func__, _dataOutput, _dataOutput, dataOutput);
    }

    if (dataOutput & LISTENER_IR_IMAGE)
    {
        listeners.irImage = new IRImageListener(this);
        royaleCamera->registerIRImageListener(listeners.irImage);
    }

    if (dataOutput & LISTENER_DEPTH_IMAGE)
    {
        listeners.depthImage = new DepthImageListener(this);
        royaleCamera->registerDepthImageListener(listeners.depthImage);
    }

    if (dataOutput & LISTENER_SPARSE_POINT_CLOUD)
    {
        listeners.sparsePointCloud = new SparsePointCloudListener(this);
        royaleCamera->registerSparsePointCloudListener(listeners.sparsePointCloud);
    }

    if (dataOutput & LISTENER_DEPTH_DATA)
    {
        listeners.depthData = new DepthDataListener(this);
        royaleCamera->registerDataListener(listeners.depthData);
    }
}

status_t TOFBridge::startCapture() { 
    M_VERBOSE("%s E\n", __func__);

    if (royaleCamera) {
        royale::CameraStatus cret = royaleCamera->startCapture();
        if (royale::CameraStatus::SUCCESS != cret) {
            M_ERROR("Royale startCapture! %d\n", cret);
            return BAD_VALUE;
        }
    }
    M_VERBOSE("%s X\n", __func__);

    return NO_ERROR;
}

status_t TOFBridge::stopCapture() {
    if (royaleCamera) {
        royale::CameraStatus cret = royaleCamera->stopCapture();
        if (royale::CameraStatus::SUCCESS != cret) {
            M_ERROR("Royale stopCapture!");
            return BAD_VALUE;
        }
    }
    return NO_ERROR;
}

void TOFBridge::getFrameRateListShortRange(std::vector<uint8_t> &list) {
    std::vector<uint32_t> *shortRange = &TOFBridge::mShortRangeFramerates;

    list.clear();
    list.insert(list.end(), shortRange->begin(), shortRange->end());
}

void TOFBridge::getFrameRateListLongRange(std::vector<uint8_t> &list) {
    std::vector<uint32_t> *longRange = &TOFBridge::mLongRangeFramerates;

    list.clear();
    list.insert(list.end(), longRange->begin(), longRange->end());
}

void TOFBridge::setFrameRate(uint8_t frameRate) {
    if (mFrameRate != frameRate) {
        M_DEBUG("%s@%d: E(frameRate= %d, currentFrameRate = %d )\n", __PRETTY_FUNCTION__, __LINE__, frameRate, mFrameRate);
        setUseCase(mDistanceRange, frameRate);
    }
}

uint8_t TOFBridge::getFrameRate() {
    uint8_t frameRate = mFrameRate;
    return frameRate;
}

void TOFBridge::setDistanceRange(RoyaleDistanceRange range) {
    if (mDistanceRange != range) {
        setUseCase(range, mFrameRate);
    }
}

RoyaleDistanceRange TOFBridge::getDistanceRange() {
    RoyaleDistanceRange range = mDistanceRange;
    return range;
}

void TOFBridge::setExposureTime(uint32_t expTime) {
    if (expTime != mExposureTime) {
        if ( expTime < mExposureLimits.first || expTime > mExposureLimits.second) {
            M_WARN("Exposure time %d is out of range [%d -- %d]\n", expTime, mExposureLimits.first, mExposureLimits.second);
        }

        CameraStatus ret = royaleCamera->setExposureTime(expTime);
        if (CameraStatus::SUCCESS != ret) {
            M_ERROR("Failed to set exposure time %d usec, ret: %d\n", expTime, ret);
            return;
        }

        setChange(EXPOSURE_TIME);
    }
}

uint32_t TOFBridge::getExposureTime() {
    return mExposureTime;
}

void TOFBridge::setExposureMode(ExposureMode expMode) {
    royale::ExposureMode mode = static_cast<royale::ExposureMode>(expMode);
    if (mode != mExposureMode) {
        CameraStatus ret = royaleCamera->setExposureMode(mode);
        if (CameraStatus::SUCCESS != ret) {
            M_ERROR("Failed to set exposure mode %d, ret: %d\n", mode, ret);
            return;
        }

        mExposureMode = mode;
        setChange(EXPOSURE_MODE);
    }
}

ExposureMode TOFBridge::getExposureMode() {
    ExposureMode expMode = static_cast<ExposureMode>(mExposureMode);
    return expMode;
}

bool TOFBridge::getChange(RoyaleParamChange param) {
    std::lock_guard<std::mutex> guard (paramChangeLock);
    uint8_t mask = static_cast<RoyaleParamChange>(param);
    bool change = ((paramChange & mask) == mask);
    return change;
}

void TOFBridge::clearChange(RoyaleParamChange param) {
    std::lock_guard<std::mutex> guard (paramChangeLock);
    uint8_t mask = static_cast<RoyaleParamChange>(param);
    paramChange &= ~mask;
}

// -----------------------------------------------------------------------------------------------------------------------------
// TofInterface class implementation 
// -----------------------------------------------------------------------------------------------------------------------------
std::vector <uint32_t> TOFBridge::mShortRangeFramerates;
std::vector <uint32_t> TOFBridge::mLongRangeFramerates;
std::vector <uint32_t> TOFBridge::mExtraLongRangeFramerates;

TOFInterface::TOFInterface(TOFInitializationData* pTOFInitializationData) {
    uint32_t                    numDataTypes = pTOFInitializationData->numDataTypes;
    RoyaleListenerType*         pDataTypes   = pTOFInitializationData->pDataTypes;
    IRoyaleDataListener*        pListener    = pTOFInitializationData->pListener;
    uint32_t                    frameRateDes = pTOFInitializationData->frameRate;  //desired frame rate and range
    RoyaleDistanceRange         rangeDes     = pTOFInitializationData->range;
    int32_t                     cameraId     = pTOFInitializationData->cameraId;

    std::vector<uint8_t>        list_short;
    std::vector<uint8_t>        list_long;
    uint8_t                     range;
    uint8_t                     type;
    uint32_t                    fps;
    uint32_t                    exp_time;
    std::pair<int64_t, int64_t> exp_time_limits;


    TOFBridge::populateSupportedUseCases(list_short, list_long, range, type, exp_time_limits, fps, exp_time);

    if (! (m_pTofBridge = new TOFBridge())) {
        M_ERROR("Can't create instance of TOF functionality for opened depth sensor\n");
        throw -EINVAL;
    }

    m_pTofBridge->cameraId = cameraId;
    m_pTofBridge->addRoyaleDataListener(pListener);

    if (m_pTofBridge->setup()) {
        M_ERROR("Could not set TOF usecase\n");
        throw -EINVAL;
    }

    M_VERBOSE("\nSetting use case: mode=%d, fps = %d\n",(int)rangeDes,frameRateDes);
    if (m_pTofBridge->setUseCase(rangeDes,frameRateDes)) {
        M_ERROR("TOFInterfaceImpl-ERROR: Could not set TOF use case!\n");
        throw -EINVAL;
    }  

    for (uint32_t i = 0; i < numDataTypes; i++) {
        m_pTofBridge->setInitDataOutput(RoyaleListenerType(pDataTypes[i]));
    }
    m_pTofBridge->startCapture();

}

#endif // QRB5165
