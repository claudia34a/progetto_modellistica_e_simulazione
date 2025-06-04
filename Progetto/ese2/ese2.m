%la matrice co2-emissions-per-capita contiene i dati filtrati dal 1900 al 2022
%delle medie prodotte a livello globale per capita di emissioni di C02,
%analogamente per la tabella WorldTemperature abbiamo filtrato le
%temperature globali dal 1900 al 2022
%La matrice tempXCO2emissions contiene 3 colonne, la prima contenente l'anno,
%la seconda contenente la temperatura, la terza contenente le emissioni di
%CO2 dell'anno corrispondente

%fonti: https://ourworldindata.org/co2-and-greenhouse-gas-emissions

clear all
load TempXCO2emissions.mat

dati=iddata(dataMatrix(:,2),dataMatrix(:,3));
%prendo righe dispari per l'identificazione e pari per la validazione
%Ottima scelta per considerare correttamente il passare degli anni sia per
%id che per val
dati_id=dati(1:2:end,:);
dati_val=dati(2:2:end,:);

na=3; %ordine parte autoregressiva
nb=3;%ordine parte esogena
nk=1;%ritardo parte esogena: 1 anno

%% controllo che na nb e nk vadano bene:
Tc=1;
max_ord_ar=3/Tc; 
max_ord_ex=3/Tc;
rit=1;%devo testare tutti i modelli che posso avere per prevedere la ia uscita
cont=0;
orizzonte_previsione=20/Tc; %Provare massimo 1 3 5
for iar=1:max_ord_ar
    for iex=1:max_ord_ex %nei problemi di previsione potrei non avere l'input
        cont=cont+1; %fpe diventa un vettore
        na=iar;
        nb=iex;
        nk=rit;
        modello=arx(dati_id, [na,nb,nk]);
        lista_modelli{cont}=modello; 
        struttura(cont,:)=[na,nb,nk];
        %devo selezionare il modello migliore 
        %barra di caricamento
        percentuale_avanzamento=(cont/((max_ord_ar)*(max_ord_ex))*10^2);
        fprintf("Avanzamento: %.2f%%\n",percentuale_avanzamento);
  
        d=na+nb;
        L=length(dati_id.y);
        %sotto il fatto di mettere orizzontale_previsione è solo perchè
        %casualmente coincide con 5s che il tempo che serve a noi
        dati_out_id=predict(modello, dati_id,orizzonte_previsione);%per fpe
        %calcolo fpe ma non lo uso perchè non ci interessa il problema
        %complessità/prestazioni
        e=dati_out_id.y-dati_id.y; %ci serve solo l'uscita dei nostri dati
        fpe=e'*e/L*(1+d/L)/(1-d/L); %e:vettore dei residui, d:numero dei parametri del modello, L: il numero dei dati

        dati_out_val=predict(modello, dati_val,orizzonte_previsione);%per mae non dati_val
        e=dati_out_val.y-dati_val.y;
        e_me=mean(e);
        e_mae(cont)=mean(abs(e));
    end
end
[min_fpe, imin]=min(fpe);
[min_me, indice_min_me]=min(e_me);
[min_mae, indice_min_mae]=min(e_mae);
modello_migliore=lista_modelli{indice_min_mae};
modello_migliore_fpe=lista_modelli{imin};
%%
%ora stampo mae del modello migliore per fpe
mae_fpe=e_mae(imin);
min_mae;
%si nota come il mae del modello migliore basandosi su fpe sia peggiore
%del modello migliore basandosi su mae
%mae_fpe = 0.1993
%min_mae = 0.0801

%% continuo esercizio
%prendo modello migliore e faccio il predict
struttura(indice_min_mae,:)

ypredict=predict(modello_migliore, dati_val,orizzonte_previsione)

%per valutare se è un buon modello mostro il mae, nme e nmae e corr
min_me %ME
min_mae %MAE
NME=min_me/mean(dati_val.y)%normalized mean error
NMAE=min_mae/mean(dati_val.y) % normalized mean absolute error
corr=corrcoef(dati_out_val.y,dati_val.y); %coefficiente di correlazione
corr=corr(1,2)

%modelli calcolati: basandoci su MAE max_ord_ar=25/Tc max_ord_ex=25/Tc
%rit=1 orizzonte=20/Tc
%na=19 nb=5 nk=1
%Discrete-time ARX model: A(z)y(t) = B(z)u(t) + e(t)                                               
% 
% A(z) = 1 - 0.3828 z^-1 - 0.4115 z^-2 + 0.04351 z^-3 - 0.5841 z^-4 + 0.163 z^-5 + 0.07409 z^-6   
%         - 0.1564 z^-7 + 0.1551 z^-8 - 0.2611 z^-9 + 0.03613 z^-10 + 0.2626 z^-11 + 0.1171 z^-12 
%         - 0.06142 z^-13 - 0.08986 z^-14 + 0.2356 z^-15 + 0.01092 z^-16 + 0.2713 z^-17           
%                                                                  - 0.2728 z^-18 - 0.2781 z^-19  
% 
% B(z) = -1.274e-09 z^-1 - 1.948e-09 z^-2 + 2.468e-09 z^-3 + 1.515e-09 z^-4 + 1.777e-09 z^-5
%min_me=-0.0428
%min_mae=0.0801
%nme=-1.0444
%Nmae=1.9552
%corr=0.5617

%modelli calcolati: basandoci su FPE max_ord_ar=25/Tc max_ord_ex=25/Tc
%rit=1 orizzonte=20/Tc
%1 na=1 nb=1 nk=1

% Discrete-time ARX model: A(z)y(t) = B(z)u(t) + e(t)
%   A(z) = 1 - 0.9468 z^-1                           
% 
%   B(z) = 3.227e-10 z^-1 

%mae=0.1993

%% mostro temperatura reale e temperatura predetta
figure(1);clf
anni=linspace(dataMatrix(1,1),dataMatrix(end,1),length(dati_val.y))';
plot(anni, dati_val.y,'LineWidth',2, 'DisplayName', 'Temperatura reale');
hold on
plot(anni, ypredict.y,'LineWidth',2, 'DisplayName', 'Temperatura predetta');
grid on
legend('-dynamiclegend','Location', 'best')
xlabel(sprintf('Anni dal %d al %d', dataMatrix(1,1), dataMatrix(end,1)));
title('Analisi nel tempo del modello migliore ')
ylabel('Anomalia della temperatura media globale')

%% Mostro errori nel grafico
figure(2); clf
plot(anni,ypredict.y-dati_val.y,'LineWidth',2,'DisplayName','Errori')
legend('-dynamiclegend','Location', 'best')
grid on
xlabel('Anni')
ylabel('Errori in gradi Celsius')
title(['MAE=',num2str(min_mae),'   NMAE=',num2str(NMAE),'   NME=',num2str(NME),'   Correlazione=',num2str(corr)])

