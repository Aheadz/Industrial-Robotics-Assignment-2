function logger(publisher,str)
    logMsg = rosmessage(publisher);
    logMsg.Data = str;
    send(publisher, logMsg);
end