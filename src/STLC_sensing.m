function [ Wn ] = STLC_sensing( Sys )
%% Make data nb_stages times longer


nw = Sys.nw;
time = Sys.time;
time_d = Sys.time_d;
Wref = Sys.Wref;

for iwx=1:nw
    Wn(iwx,:) = interp1( time , Wref(iwx,:)', time_d)';
end
end

