
-- Versionando esse arquivo devido a falha que impossibilita atualizar o arquivo da simulacao em alguns computadores
-- Este script deve ser incluindo no 'child script' do elemento Base_PS da simulacao.

function sysCall_init()
    
    -- do some initialization here
    arm=sim.getObjectHandle("Arm_actuator")
    crab=sim.getObjectHandle("Crab_actuator")
    hoist=sim.getObjectHandle("Hoist_actuator")
    rotate=sim.getObjectHandle("Revolute_hoist")
    suction=sim.getObjectHandle('suctionPad')
    
    xml = [[
        <ui title="Speed Control" closeable="true"  resizable="false" activate="false">
            <group layout="form" flat="true">
                <label text="Arm speed (rad/s): 0.00" id="1"/>
                <hslider tick-position="above" tick-interval="1" minimum="-10" maximum="10" on-change="actuateArm" id="2"/>
                <label text="Crab speed (m/s)" id="3"/>
                <hslider tick-position="above" tick-interval="1" minimum="-10" maximum="10" on-change="actuateCrab" id="4"/>
                <label text="Hoist speed (m/s)" id="5"/>
                <hslider tick-position="above" tick-interval="1" minimum="-10" maximum="10" on-change="actuateHoist" id="6"/>
                <label text="Revolute Hoist speed (rad/s): 0.00" id="7"/>
                <hslider tick-position="above" tick-interval="1" minimum="-10" maximum="10" on-change="rotateHoist" id="8"/>
                <label text="Magnet" id="9"/>
                <button text="Not Active" on-click="actuateMagnet" checkable="true" id="10"/>
            </group>
            <label text="" style="* {margin-left: 400px;}"/>
        </ui>
    ]]

    ui=simUI.create(xml)
end

function actuateArm(ui, id,newVal)
    val = -0.02*newVal
    sim.setJointTargetVelocity(arm,val)
    simUI.setLabelText(ui,1,string.format("Arm speed (rad/s): %.2f",-val))
end

function actuateCrab(ui, id, newVal)
    local val = 0.05*newVal
    sim.setJointTargetVelocity(crab,val)
    simUI.setLabelText(ui,3,string.format("Crab speed (m/s): %.2f",val))
end

function actuateHoist(ui, id, newVal)
    local val = -0.25*newVal
    sim.setJointTargetVelocity(hoist,val)
    simUI.setLabelText(ui,5,string.format("Hoist speed (m/s): %.2f",-val))
end

function rotateHoist(ui, id,newVal)
    val = -0.02*newVal
    sim.setJointTargetVelocity(rotate,val)
    simUI.setLabelText(ui,7,string.format("Revolute Hoist speed (rad/s): %.2f",-val))
end

function actuateMagnet()
    local state = sim.getScriptSimulationParameter(suction,'active')
    if state then
        sim.setScriptSimulationParameter(suction,'active','false')
        simUI.setButtonText(ui, 10, "Active")
    else 
        sim.setScriptSimulationParameter(suction,'active','true')
        simUI.setButtonText(ui, 10, "Not Active")
    end
end