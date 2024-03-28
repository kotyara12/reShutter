# reShutter: управление электроприводом с реверсом и промежуточными состояниями

Библиотека **reShutter** создана для управления простыми электроприводами с возможностью реверса и промежуточными состояниями - кранами, форточками, заслонками и т.д. 

Библиотека рассчитана на приводы с обычными коллекторными или безколлекторными электродвигателями без пошагового управления, поэтому все управление осуществляется на основе временных интервалов (программных таймеров). То есть для открытия или закрытия привода в определенное положение включается привод на опреденное время. Для осуществления реверса используются два разных GPIO, поэтому используйте мостовую схему для непосредственного управления двигателем. 

_При работе библиотеки предполагается, что она никак не контролирует текущее физическое положение привода, а отключение электродвигателя в кончных положениях осуществляется с помощью встроенных в привод конечных выключаетелей_. Поэтому при иницициализации экземпляра класса привод всегда переводится в положение "_полностью закрыто_", а затем отчитываем положение исходя из этого начального положения. Поэтому перед использованием библиотеки важно максимально точно определить время полного закрытия или открытия привода. Однако вы можете добавить "внешний" контроль положения с помощью каких-либо датчиков самостоятельно.

Библиотека реализована в виде нескольких классов, которые поддерживают работу как с встроенными GPIO микроконтроллера, так и с расширителями GPIO типа PCF8574 и аналогичных.