#pragma once

#include "Common/spcontext.h"

#include "protocol/protocol.h"

#include "common/AppAlert.h"

#include <string.h>

#include <QDataStream>

#include "project/project.h"

#include <mutex>

#include "serial/phasedata.h"

class SerialPort : public QObject{
    Q_OBJECT
public:

    static SerialPort *GetInstance();

    static QByteArray _buffData;

    static int _buffData_len;

    static int serialFd;

    static void WriteData(SPContext &context);

    static void WriteRawData(int no,AsyncType asyncType,Request &request,CallBackController callBackController);

    static void WriteHeartbeat();

    static void OnRecvData(char * data,int len);

    static void dealRecvData(const char*,int len);

    static void dealRecvRawData(const char*,int len);

    static void dealRecvFileData(const char*,int len);

    static QByteArray restoreProjectFromFile();

    static QByteArray restoreConfigFromFile();

    static void restoreParamFromFile();

    static void saveParamToFile(QMap<int,QString>);

    static QByteArray dealSendData(SPContext &context);

    static void setPhaseDataUtil();


private:

    SerialPort();

    ~SerialPort();

    SerialPort(const SerialPort&);

    SerialPort& operator=(const SerialPort &);



    static std::string trim_back(std::string str);



    QByteArray SerializeData(SPContext &context);



    static std::mutex mutex_;

    static std::mutex rec_mutex_;

    static SerialPort *m_instance;

    static PhaseData* phaseData;

};
