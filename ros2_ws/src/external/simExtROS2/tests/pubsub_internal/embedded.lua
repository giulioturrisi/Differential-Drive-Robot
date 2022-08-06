--@mode child

function simulationCount()
    local t=sim.unpackInt32Table(sim.readCustomDataBlock(sim_handle_app,'@tmpsimulationCount'))
    return t[1]
end

function cb(msg)
    if msg.data==topicdata then
        local c=simulationCount()
        simTest.logInfo('received correct data at run '..c)
        if c>2 then simTest.success() end
        sim.stopSimulation()
    end
end

function sysCall_init()
    require 'simTest'
    topicname='/simtest123'..math.floor((1+math.random())*1e9)
    topictype='std_msgs/msg/Int32'
    topicdata=math.floor((1+math.random())*1e9)
    pub=simROS2.createPublisher(topicname,topictype)
    sub=simROS2.createSubscription(topicname,topictype,'cb')
end

function sysCall_sensing()
    local msg=simROS2.createInterface(topictype)
    msg.data=topicdata
    simROS2.publish(pub,msg)
    if sim.getSimulationTime()>2 then
        simTest.failure('did not receive any data')
    end
end

function sysCall_cleanup()
end
