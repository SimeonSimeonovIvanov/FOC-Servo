# FOC-Servo

[FOC Servo (youtube.com)](https://www.youtube.com/watch?v=d91JvBRgOYI&list=PLE616v1yP137koahQjisksADdZlHxYS2S)<br>
[Images (photos.google.com)](https://goo.gl/photos/JQcb6tujQgFE7cGe8)<br>

![Screen Shot](https://lh3.googleusercontent.com/vWZbaQVdAhu1QQO_cTlWY4MgnAb2BmGCg2ht9W3ZADUsB-J3ieZKUOjoW8N4zHFRGdZp8hFGPKCy6uawGKFupMPxBJmIFaj5hGluz3LkEmy5S17hmuiLZEQlVxxbsq9VOWg288KpUEtcyImdLcbH4nJbDOrk7feC0b6LQ1DTAXJD6xIRRkscn5fKtOFiCyshHNf-yYZnXZzY7v92_SR10B9enX9Oa1QhKC0Zo4pFHIeS9buX3Prl2VSoP0p2f9dSjYG3iJzVJJ4cjOTOYSJCoFbktkBBuRvZ9yOYemuwcPBqjm2Aqjuo5mWAgzC-g0u8LBz8_ISS-xWrEu7XkNp3rqmE0j6_iL28L1pHF4nmJSnV0F_0pvuBMTZbZnvaV3lmYf0SQIsa0-76etPjSC5jA3zWO_9prSDMudj7b2m4mwxrd_XN79okazN7x6W9LXKWux-t0f8W2QBu25lEgwM-dTigUHxHYiCUhdcBQTDeFgdGpegSvgsOzxQlizkStA1ZrNoxv37GxwSWOPVyg8SRRRMWrbqQP0fdLs4uiXirrpl6c84sBvOwoKhNLHmpYTo9-d7bUEOs_W8uyZnnRY8boEB0MJfAz7LJxCBVNjXnpN_daKvdNwv219_3wybdyhxH5bj7qB9RDaEJVHH0xf9plpmDgyaLtqyamIorE5iW7Q=w1572-h1179-no)<br>
![Screen Shot](https://lh3.googleusercontent.com/k02gDKrigeXRs4gnt29zIekYWgJM7NDDvpPBwwBkn84v4wwCzqt8nw-INf11CDgn7xQ7Py5mxIiK_7V0SbNQoPlOT3bTszP4-zwKSuMI01aGK7_4IhnsZZ5Z7r1dPS8m6I9LZv3YYpQsvTk1K4j1EelQm8jLoi5QNvEOz-FvpGVZwKJkS9vO1XtkHeE4zk9rVJoZjloqsFnxf6_mEEpo1JaTmMSzqUu0BTOOYX97MykgnaL4p3ELG66Kbez4t6Lvg8YSnU9Ltbq5jz9uCMgsXV7kkRO0mRdbd6jPAT2u14GJWXknMUGbn2uqpa3rRhNTl81p2y2OLagABrOaGPtVqzI5KLYQBMGvRb8pLV3gTC-SgxF0fuM8P7-ImlSoCKKW4M-aQfWppRQZcFfsLp-E7-kPCjrrwxV5XEZZyoRGvc7qfGslpKFDynpH2KSDDe61FRB4h4jSAbreOhekBrllv7LVYmnvaclYAikYPAygZMUPMjnbqFKZi_-_RrciDLW16teUsW3NS0HijvG6KOLEbynRKwrUGT3R_amJrg2y10YHOSGbepvXF0Xuh3haF8Pk2yhVCZG2qWtMv-iXLtL526J6DCeSrQHVDLIhtxiESPV2ZTU_wgwGWl188nJnMD6YXfbKLWg5SoCA2hz2HVpunoGfktz3Lt1SIodtHRgSmA=w1572-h1179-no)<br>

http://mcu-bg.com/mcu_site/viewtopic.php?p=226271#p226271:<br>
"След аналоговото DC-Servo (viewtopic.php?f=22&t=13129) започнах работа по BLDC-Servo. Работих по хардуерен синусоидален контрол на AC Servo motor (PMSM) с три токови ШИМ-а. Успях да постигна токова регулация в 6 стъпкова комутация. Синусоидален режим така и не подкарах напълно. Но в крайна сметка това беше само за упражнение. Целта през цялото време беше векторно управление (FOC) на мотора. Самия мотор претърпя малка реконструкция - изведох сигналите за комутация UVW навън през RS422 предавател.

В крайна сметка стигнах до FOC-Servo. За момента имам реализирано векторно управление на базата на STM32F4. Липсва обратна връзка по скорост. Директно задавам позиция от импулсен генератор. Работи добре, има още какво да се желае.

Редно е да отбележа, че токовата обратна връзка минава през модули от честотни задвижвания на Electroinvent. Също и импулсното захранване на електрониката е от компоненти / схема (свалена от платка) на тяхно задвижване. Първоначално силовия мост беше с импулсни DC/DC преобразуватели но в случай на пробив на транзистор си 'заминаваха' набързо. Също SVPWM е копиран от кодa на STM FOC SDK v1. Другите компоненти (софтуер/хардуер) са смесица от мои и чужди проекти.

Продължавам работа по задвижването, изкушавам се да бъде част от бъдещо обновяване на ROBKO-01."

http://mcu-bg.com/mcu_site/viewtopic.php?p=235621#p235621:<br>
"Вече задвижването се захранвана директно от мрежата ( 220 VAC -> 320 VDC на моста ). Изпитвах доста притеснения в етапа на увеличаване на напрежението. Разбира се имаше и проблеми. Сред които особено неприятна осцилация на GS на полевите транзистори. Успях да затворя и позиционния и скоростния контур. За жалост обратната връзка по скорост все още е прекалено груба и това създава проблеми но все пак работи доста добре.

Файловете на проекта са качени в https://github.com/SimeonSimeonovIvanov/FOC-Servo
Видео от отделните тестове: youtube.com / FOC Servo"

http://mcu-bg.com/mcu_site/viewtopic.php?p=277382#p277382:<br>
"Да се похваля :) От проекта за FOC Servo (open source) успях да достигна до нещо много по-истинско: AC Servo. Засега не съм решил дали ще премине към open или close. Отделни схемни решения от него са качени в git: https://github.com/SimeonSimeonovIvanov/"

# AC-Servo

[AC Servo (photos.google.com)](https://photos.app.goo.gl/785x7djrvYAcP9JZ6)<br>
[AC Servo ( Work ) (photos.google.com)](https://photos.app.goo.gl/srzg5DDZzGg4B8dc7)<br>

![Screen Shot](https://lh3.googleusercontent.com/InO0-X00L5HjlXPxuh1YEjHJFSkUZFtOApBEi4UaxSTPb9P_QFyURlaM9I8N8cGii7pULpo6o4EXxeGouaYiY4lC6Kz6RivBfqiPAh9K2oRpcYkvYIHu17OkECJ6e2FBvHD5CIEYPnccBth1VAMi6_NGUH8vluU_3zfoT785-X81UAu9X3ewuF4Hv4TFzx-BPBHCW2oewmVy1OYqN_RM4jMVyNYgm047YA81ab1-KXVNjyB3TT6EyaVp_5hVKxsnxAwqsmA9SToYOupK6zUkhS5GmJsAwiaa065885K3q9S96F79FyxyqpMNXp9kvOWaOszUekfrR_CrvjRdldlZTlmVMwnjsETIEQ2s5GbruYBjzTjdpzQbDMcVgDvs_bUZGg7GsPLNmPTYXevoz2cMCvbN2Q-9_fhSFlpYpl3opcZ2RupjmNxaqhT3yl_42L_v6f9WMqQT6NlQBk_QC21Ln013LTSn60Qf7zM8eraPTgBrA7T2kTm2WHEvtyGzwhygOm-KNEVWI5vc4HmTfOWSWKSE_RJDDSoxtdrD4LYK1ZHu1DPog4PJUM_owncTiQRPLKI1CUuruEjPLMy4CZy9KJe3nysguwvQm7qaSPnMRbYmxLYDbtbTNJx2IUp1Rz1COHA0BrA35666Xp8BiKKPRK0gXBy38mMYAO1zs7tCIh62oI72LmdtQqa65aXLWokCe901ilnWZ3QXxQAgrQ=w1260-h945-no)<br>

![Screen Shot](https://lh3.googleusercontent.com/okfIJzOqILg2Vj1FPi2K5oUJk9owYkxtr5FeaoL3CWSu0oE8lGGliDmks5RnoqhALW29WYXMFR0nzFvj4oWKjR9dDEuZ9FIdgU9OOmerqA_8B_PouIPbHv3mtwUzUjZpZbUSutIWh9YmzeeYPLEdk_HFiIJOq9GOc4Usw812NYWofKSyBRqMkyVN_z-J87s8EabtJWR5lwjRa-cBh1tVUnRWEO9jsjj2K1f_PH0fb7r-1IhrKMD5mt5mQMLsQQ3XEQHfQMr2cBMIuz-dUbnQfps5Nz2GnchztAow77KTyyPxqd5mdj42PTM2ViCJxDiPJYyyLR3K6_ppbO3PRpn_wdOt01Bq-PtScpuafN_ghBSL42ZQjAdX2HB0avNf8NTuKXV2K2KDEkNEty0ng5hCATOcCaR4_sYTx5rRfgzS5E-67_TDmEbXEtx20I6fUW2qXE5jVsJh5uaagef236Bs2ssdnnitRStcJ5OLySjDwEt8WhWuonXRYj4Zssnljycxc0Kr6qCRuJdaS_oqwF6SjVi7_j527KhP15YrcITrhYX8e88y9OKlFIuTc17PmZ9Ysnu4XRD6lYgGd9tDdisBCfOo52TFicWKevpks9opckHmaGbfbybtolHaqSJT_40r1s_MAaIwxZIIcEhB-e_utXBml-KW7WElzk1bFA_dYzNY4fb4RSdAxStCFWRdlXUccUR_GF7sj9hS86BAyA=w1263-h947-no)<br>

Simulation:

FOC.grf.png:
![Screen Shot](https://raw.githubusercontent.com/SimeonSimeonovIvanov/FOC-Servo/master/doc/FOC.grf.png)<br>

ATAN.grf.png
![Screen Shot](https://raw.githubusercontent.com/SimeonSimeonovIvanov/FOC-Servo/master/doc/atan.grf.png)<br>

Sin/Cos Encoder:
![Screen Shot](https://raw.githubusercontent.com/SimeonSimeonovIvanov/FOC-Servo/master/src/Work/sin_cos_encoder/sin_cos_encoder.jpg)<br>

Bipolar Analog Output:
![Screen Shot](https://raw.githubusercontent.com/SimeonSimeonovIvanov/FOC-Servo/master/Scheme%20and%20Board/Work/AO/AO0%20(%20New%20).jpg)<br>
