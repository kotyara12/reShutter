# reShutter: управление электроприводом с реверсом и промежуточными состояниями

Библиотека **reShutter** создана для управления простыми электроприводами с возможностью реверса и промежуточными состояниями - кранами, форточками, заслонками и т.д. 

Библиотека рассчитана на приводы с обычными коллекторными или беcколлекторными электродвигателями без пошагового управления, поэтому все управление осуществляется на основе временных интервалов (программных таймеров). То есть для открытия или закрытия привода в определенное положение на него просто подается соответствующее напряжение на определенное время, рассчитанное библиотекой. Для осуществления реверса используются два разных GPIO, поэтому вам необходимо использовать мостовую схему для непосредственного управления двигателем. 

_При работе библиотеки предполагается, что она никак не контролирует текущее физическое положение привода, а отключение электродвигателя в кончных положениях осуществляется с помощью встроенных в привод конечных выключаетелей_. Поэтому при инициализации экземпляра класса привод всегда переводится в положение "_полностью закрыто_", а затем отсчитывается положение исходя из этого начального значения. Поэтому перед использованием библиотеки важно максимально точно определить время полного закрытия или открытия привода. Однако вы можете добавить "внешний" контроль положения с помощью каких-либо датчиков самостоятельно.

Библиотека реализована в виде нескольких классов, которые поддерживают работу как с встроенными GPIO микроконтроллера, так и с расширителями GPIO типа PCF8574 и аналогичных. 

- class __rGpioShutter__ предназначен для работы с встроенными GPIO
- class __rIoExpShutter__ предназначен для работы через расширители GPIO

Вы можете объявить несколько отдельных экземпляров для управления различными приводами в одном и том же проекте.

Дополнительную справочную информацию об использовании данной библиотеки вы можете почерпнуть из файла reShutter.h и на сайте https://kotyara12.ru