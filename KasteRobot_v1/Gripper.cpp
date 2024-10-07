#include "Gripper.h"
#include <QHostAddress>
#include <QDebug>
#include <QTimer>
#include <QEventLoop>

Gripper::Gripper(QObject *parent) :
    QObject(parent),
    socket(new QTcpSocket(this)) // Initialize the QTcpSocket
{
    // Connect signals to the slots
    connect(socket, &QTcpSocket::connected, this, &Gripper::onConnected);
    connect(socket, &QTcpSocket::disconnected, this, &Gripper::onDisconnected);
    connect(socket, &QTcpSocket::readyRead, this, &Gripper::onReadyRead);
    connect(socket, qOverload<QAbstractSocket::SocketError>(&QTcpSocket::errorOccurred), this, &Gripper::onError);
}

Gripper::~Gripper()
{
    // Clean up
    socket->disconnectFromHost();
    delete socket;
}

// ---- TCP Related ----
void Gripper::connectToServer(const QString &host, quint16 port)
{
    // Connect to the server
    qDebug() << "Connecting to server: " << host << ":" << port;
    socket->connectToHost(host, port);

    // Wait for the connection to be established or fail
    if (!socket->waitForConnected(5000)) { // wait for 5 seconds
        qDebug() << "Failed to connect to server: " << socket->errorString();
    }
}

void Gripper::disconnectFromServer()
{
    // Disconnect from the server
    if (socket->isOpen()) {
        qDebug() << "Disconnecting from server";
        socket->disconnectFromHost();
    }
}

void Gripper::sendData(const QByteArray &data)
{
    // Check if the socket is in the connected state
    if (socket->state() == QAbstractSocket::ConnectedState) {
        // Send data with a newline character if necessary
        socket->write(data + "\n");

        // Wait for data to be written
        if (!socket->waitForBytesWritten(3000)) {
            qDebug() << "Failed to send data: " << socket->errorString();
        }
        else {
            qDebug() << "Data sent successfully: " << data;
        }
    }
    else{
        qDebug() << "Socket is not connected!";
    }
}

void Gripper::onConnected()
{
    // Handle when the connection is established
    qDebug() << "Connected to server!";
    statusConnection = true; // Update connection status
}

bool Gripper::isConnected() const
{
    return statusConnection; // Return connection status
}

void Gripper::onDisconnected()
{
    // Handle when the connection is closed
    qDebug() << "Disconnected from server.";
    statusConnection = false; // Update connection status
}

void Gripper::onReadyRead()
{
    // Handle incoming data
    mReadDataArray = socket->readAll(); // Read all available data
    // Convert QByteArray to QString
    mReadDataString = QString::fromUtf8(mReadDataArray);
    mReadDataString.remove(mReadDataString.length() - 1, 2);
    //qDebug() << "Converted QString: " << mReadDataString;
}

void Gripper::onError(QAbstractSocket::SocketError socketError)
{
    // Handle socket errors
    qDebug() << "Socket error: " << socketError << " - " << socket->errorString();
}



// ---- Gripper Related ----
// Usage of the univeral class:
//
//  Find the command you want to use in the documentation for the gripper and use it and the input for the function.
//  Some commands involving complex responses from the gripper is NOT handled by this class. ONLY MOVEMENT BASED COMMANDS!
//  If in doubt how to use the universal command, please use the predefined commands.
bool Gripper::Command(QString command) {
    // Step 1: Send the command
    // Use the command provided as a parameter. This in theory can be any command the gripper can preform
    mSendData = command;
    // Convert the data to a Qbytearray
    sendData(mSendData.toUtf8());

    // Step 2: Prepare to read the response
    // Trim the command into its basic, so it can be read in the response from the gripper
    // First find the last '('
    int lastIndex = command.lastIndexOf('(');
    QString trimmedCommand;
    // If a '(' was found, trim the command
    if (lastIndex != -1) {
        trimmedCommand = command.remove(lastIndex, command.length() - lastIndex);
    }
    // If lastIndex is -1, then no '(' was found and the trimmed command is the same as the command
    // Should ideally never happen
    else {
        trimmedCommand = command;
    }
    //Construct the expected responses, setup flags for monitoring and objects for the event loop
    QString expectedAck = "ACK " + trimmedCommand;
    QString expectedFin = "FIN " + trimmedCommand;
    QEventLoop loop;
    QTimer timer;
    bool ackReceived = false;
    bool finReceived = false;

    // Step 3: Connect signals to handle incoming data in a lamda function
    // Every time the readyRead is triggered in socket, the function triggers with all the varaibles in scope [&].
    // So if the gripper pre
    connect(socket, &QTcpSocket::readyRead, this, [&]() {
        qDebug() << "Received response: " << mReadDataString;

        // Check for "ERR" in the response, quit the event loop, since no futher response will be recived.
        if (mReadDataString.contains("ERR")) {
            qDebug() << "Error detected in response: " << mReadDataString;
            loop.quit();
            return;
        }

        // Check for expected ACK response
        if (mReadDataString.trimmed() == expectedAck) {
            qDebug() << "Command " << trimmedCommand << " Acknowledged";
            ackReceived = true;
        }

        // Check for expected FIN response
        if (mReadDataString.trimmed() == expectedFin) {
            qDebug() << "Command " << trimmedCommand << " Finished";
            finReceived = true;
        }

        // If both ACK and FIN are received, exit the loop
        if (ackReceived && finReceived) {
            loop.quit();
        }
    });

    // Step 4: Setup a timeout mechanism. Singleshot timer of 20 seconds
    timer.setSingleShot(true);
    timer.start(20000);

    // Step 5: Connect the timer timeout signal to quit the event loop
    connect(&timer, &QTimer::timeout, &loop, [&]() {
        qDebug() << "Timeout waiting for response";
        loop.quit();
    });

    // Step 6: Start the event loop and wait for either response or timeout!
    loop.exec();

    // Step 7: Return 1 if both reponsed is receive, or 0 if not
    if (ackReceived && finReceived) {
        loop.quit();
        return 1;
    }
    else {
        loop.quit();
        return 0;
    }
}

// ---- Functions for General (For noobs) ----
bool Gripper::Home(){
    // Call the Command function to home. Return result.
    // Defualt call
    return Command("HOME()");
}

// ---- Functions for Gripping (For noobs) ----
bool Gripper::Grip(){
    // Call the Command function with the force 10, and size 40. Return result.
    // Defualt call
    return Command("GRIP(10, 40)");
}
bool Gripper::Grip(int force){
    // Call the Command function with just a force int
    return Command("GRIP(" + QString::number(force) + ")");
}
bool Gripper::Grip(int force, int size){
    // Call the Command function with just a force int and a size
    return Command("GRIP(" + QString::number(force) + "," + QString::number(size) + ")");
}
// ---- Functions for releasing (Also for noobs) ----
bool Gripper::Release(){
    // Call the command function with the release function, default values for the gripper
    return Command("RELEASE()");
}
bool Gripper::Release(int pullback){
    // Call the command function with the release function, with desired pullback
    return Command("RELEASE(" + QString::number(pullback) + ")");
}




