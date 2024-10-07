#ifndef GRIPPER_H
#define GRIPPER_H

#include <QObject>
#include <QTcpSocket>
#include <QString>

class Gripper : public QObject
{
    Q_OBJECT

public:
    explicit Gripper(QObject *parent = nullptr);
    ~Gripper();

    // -- TCP Related --
    // Connects to the server at the given host and port
    void connectToServer(const QString &host, quint16 port);
    // Disconnects from the server
    void disconnectFromServer();
    // Sends raw data to the server
    void sendData(const QByteArray &data);
    //Function to check the state of statusConnection
    bool isConnected() const;

    // -- Gripper Related --
    // Function for universal command of the gripper
    bool Command(QString command);
    // Functions for Gripping
    bool Grip();
    bool Grip(int force);
    bool Grip(int force, int size);
    // Functions for releasing
    bool Release();
    bool Release(int pullback);


private slots:
    // Handles when the socket successfully connects
    void onConnected();
    // Handles when the socket disconnects
    void onDisconnected();
    // Handles incoming data from the server
    void onReadyRead();
    // Handles socket errors
    void onError(QAbstractSocket::SocketError socketError);

private:
    QTcpSocket *socket;
    // Bool for connection status
    bool statusConnection;
    // Member varaiables for sending and reading data
    QString mSendData;
    QByteArray mReadDataArray;
    QString mReadDataString;
};

#endif // GRIPPER_H

