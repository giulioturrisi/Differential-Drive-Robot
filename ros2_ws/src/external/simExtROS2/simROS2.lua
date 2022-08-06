local simROS2 ={}

--@fun timeFromFloat
--@arg float t the time as a floating point number
function simROS2.timeFromFloat(t)
    local r={}
    r.sec=math.floor(t)
    r.nanosec=math.floor(1e9*(t-r.sec))
    return r
end

--@fun timeToFloat
--@arg table t the time as a table with integer sec and nanosec fields
function simROS2.timeToFloat(t)
    return t.sec+1e-9*t.nanosec
end

--@fun getSystemTime
function simROS2.getSystemTime()
    return simROS2.timeFromFloat(sim.getSystemTime())
end

--@fun getSimulationTime
function simROS2.getSimulationTime()
    return simROS2.timeFromFloat(sim.getSimulationTime())
end

--@fun importInterface
--@arg string name the name of the interface to import, e.g.: geometry_msgs/msg/Vector3
function simROS2.importInterface(name)
    local name_comp={}
    for part in string.gmatch(name, "[^/]+") do
        table.insert(name_comp, part)
    end
    if #name_comp ~= 3 then
        return nil
    end
    local if_def=simROS2.getInterfaceConstants(name)
    if_def.__name=name
    if_def.__new=function() return simROS2.createInterface(name) end
    if _G[name_comp[1]] == nil then _G[name_comp[1]]={} end
    if _G[name_comp[1]][name_comp[2]] == nil then _G[name_comp[1]][name_comp[2]]={} end
    _G[name_comp[1]][name_comp[2]][name_comp[3]]=if_def
end

function simROS2.log(verbosity,msg)
    sim.addLog(verbosity,msg)
end

function simROS2.__renamed(old,new)
    simROS2.log(sim.verbosity_scripterrors,'simROS2.'..old..' has been renamed to simROS2.'..new)
end

function simROS2.subscribe(topicName,topicType,callback)
    simROS2.__renamed('subscribe','createSubscription')
end

function simROS2.shutdownSubscriber(topicName,topicType,callback)
    simROS2.__renamed('shutdownSubscriber','shutdownSubscription')
end

function simROS2.subscriberTreatUInt8ArrayAsString(topicName,topicType,callback)
    simROS2.__renamed('subscriberTreatUInt8ArrayAsString','subscriptionTreatUInt8ArrayAsString')
end

function simROS2.advertise(serviceName,serviceType,callback)
    simROS2.__renamed('advertise','createPublisher')
end

function simROS2.serviceClient(serviceName,serviceType,callback)
    simROS2.__renamed('serviceClient','createClient')
end

function simROS2.shutdownServiceClient(serviceName,serviceType,callback)
    simROS2.__renamed('shutdownServiceClient','shutdownClient')
end

function simROS2.serviceClientTreatUInt8ArrayAsString(serviceName,serviceType,callback)
    simROS2.__renamed('serviceClientTreatUInt8ArrayAsString','clientTreatUInt8ArrayAsString')
end

function simROS2.advertiseService(serviceName,serviceType,callback)
    simROS2.__renamed('advertiseService','createService')
end

function simROS2.shutdownServiceServer(serviceName,serviceType,callback)
    simROS2.__renamed('shutdownServiceServer','shutdownService')
end

function simROS2.serviceServerTreatUInt8ArrayAsString(serviceName,serviceType,callback)
    simROS2.__renamed('serviceServerTreatUInt8ArrayAsString','serviceTreatUInt8ArrayAsString')
end

function simROS2.imageTransportSubscribe(serviceName,serviceType,callback)
    simROS2.__renamed('imageTransportSubscribe','imageTransportCreateSubscription')
end

function simROS2.imageTransportShutdownSubscriber(serviceName,serviceType,callback)
    simROS2.__renamed('imageTransportShutdownSubscriber','imageTransportShutdownSubscription')
end

function simROS2.imageTransportAdvertise(serviceName,serviceType,callback)
    simROS2.__renamed('imageTransportAdvertise','imageTransportCreatePublisher')
end

return simROS2
