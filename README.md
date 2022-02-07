# PotentialFieldsNav

Sviluppo di un nodo ROS (Robot Operating System) per la navigazione autonoma del robot in ambienti indoor realizzata tramite l’applicazione di campi di forza attrattivi e repulsivi.

Nell’ottica di una metodologia “Sense Plan Act” il progetto si pone inizialmente lo scopo di rappresentare l’ambiente circostante tramite una mappa e di mantenere informazioni riguardanti gli ostacoli all’interno di essa associandogli le rispettive forze repulsive e, successivamente, di configurare la forza attrattiva che permetta al robot di trovare un percorso verso il “goal”, ovvero l’obiettivo richiesto.

Sperimentazione:
- Esperimenti con simulatore:  nodo stageros del framework ROS contenuto all’interno del package stage_ros
- Esperimenti in ambiente reale:  robot differential drive MARRtino
