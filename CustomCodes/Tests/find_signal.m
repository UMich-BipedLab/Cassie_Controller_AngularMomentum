function find_signal(abus)
global bus_signal_count
global bus_signals
load('buses.mat');
    for i = 1:length(abus.Elements)
%         if isa(abus.Elements(i),'Simulink.Bus')
        if strcmp(abus.Elements(i).DataType(1:4),'Bus:')
            newbus_name = abus.Elements(i).DataType(6:end);
            eval(['new_bus = ' newbus_name ';']);
            find_signal(new_bus);
        else
            bus_signal_count = bus_signal_count+1;
            signal = abus.Elements(i);
            bus_signals{bus_signal_count}=signal;
            
        end
    end
end

