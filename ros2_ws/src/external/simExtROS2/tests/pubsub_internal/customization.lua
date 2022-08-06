--@mode customization

function sysCall_init()
    require 'simTest'
    simulationCount=0
end

function sysCall_nonSimulation()
    simulationCount=simulationCount+1
    sim.writeCustomDataBlock(sim_handle_app,'@tmpsimulationCount',sim.packInt32Table{simulationCount})
    sim.startSimulation()
end

function sysCall_sensing()
    if sim.getSimulationTime()>4 then
        simTest.logInfo('timeout reached')
        simTest.failure()
    end
end
