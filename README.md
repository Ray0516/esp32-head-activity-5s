# ESP32 頭戴式活動狀態辨識（5 秒輸出）

這是 ESP32 + MPU6050 的獨立韌體。六軸資料、轉譯、濾波、時間窗特徵、活動指紋資料庫與 21 棵多分枝決策樹都在 ESP32 端執行；接收端不需要再做辨識。

## 最終狀態

| 等級 | JSON 狀態 | 說明 |
|---:|---|---|
| 0 | `still` | 靜止 |
| 1 | `light` | 輕度活動 |
| 2 | `moderate` | 中度活動 |
| 3 | `high` | 高度活動 |

MPU6050 仍以 100 Hz 連續取樣，模型每 1 秒重新判斷一次，只有最終狀態固定每 5 秒輸出一次，所以不會用大量波形資料塞住序列埠。

## 完整處理流程

1. MPU6050 設定為 `±4 g`、`±500 °/s`、100 Hz FIFO。
2. 將三軸加速度與三軸角速度轉成模型使用的 `g` 與 `°/s`。
3. 扣除開機陀螺儀偏移，套用 5 Hz 低通濾波。
4. 在來源相容的 10 秒滑動資料窗計算六個特徵：加速度向量的平均、標準差、變異數，以及角速度向量的平均、標準差、變異數。
5. 完整走訪 `auritus_activity_forest.h` 內的 21 棵樹；節點總數記錄在 `MODEL_DATABASE.json`。
6. 對四個可輸出活動類別投票，再加入不改動原始資料庫的可調偏移。
7. 連續兩次得到同一個新結果才切換狀態，降低瞬間晃動造成的跳級。
8. 每 5 秒輸出一筆 JSON。

## 接線

| MPU6050 | ESP32 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

## 建置與燒錄

以 VS Code + PlatformIO 開啟本資料夾：

```text
pio run
pio run --target upload
pio device monitor --baud 230400
```

## 正常輸出

```json
{"type":"activity_state","version":1,"firmware":"head-activity-5s-v1","seq":12,"t":60000,"sensor":true,"valid":true,"activity":"moderate","activityLevel":2,"confidence":0.762,"ttlMs":6500}
```

- `sensor`：最近是否持續收到 MPU6050 資料。
- `valid`：模型是否已有足夠資料可判斷。
- `activity` / `activityLevel`：機器使用的狀態名稱與 0–3 等級。
- `confidence`：獲勝類別在 21 棵樹中的原始投票比例。
- `ttlMs`：接收端超過 6.5 秒未收到新狀態時應視為失聯。

若資料停止，韌體會清除舊時間窗、送出 `valid:false`，並自動執行 I2C bus clear 與 MPU6050 重新初始化，不需要靠頁面重開。

開機後前 10 秒是暖機期；仍會每 5 秒輸出暫定狀態，完整資料窗後 `valid` 才會變成 `true`。

## 模型資料

- 來源：[nesl/auritus](https://github.com/nesl/auritus) 頭戴／耳戴式慣性感測資料。
- `src/auritus_activity_forest.h`：四種活動指紋、正規化數值、21 棵樹的全部根節點、分枝與葉節點。
- `MODEL_DATABASE.json`：來源檔雜湊、欄位對應、資料筆數、固定種子、測試結果與匯出檔雜湊。
- 原始 CSV 保持唯讀；ESP32 使用的是轉譯後放在 Flash 的 C++ 資料表。

## 模型資料與後續設備整合說明

本專案未來可再連接馬達、風量控制器、通訊模組或其他設備，但活動辨識模型與指紋資料庫是**唯讀的基準資產**，不屬於設備控制層。任何 AI、程式產生器或後續開發者都必須遵守以下規則：

1. 不得修改、重建、刪減或覆寫 `src/auritus_activity_forest.h`、`MODEL_DATABASE.json` 與 `FINGERPRINTS.md` 的模型資料。
2. 不得用固定狀態、亂數、範例數字、假投票結果或接收端推測，冒充 ESP32 決策樹的輸出。
3. `activity`、`activityLevel`、`confidence` 與 `valid` 必須來自 ESP32 對 MPU6050 實際取樣後，完整走訪 21 棵決策樹所得的結果。
4. 馬達與其他設備只能讀取每 5 秒輸出的最終狀態，並在獨立的控制層決定動作；控制層不得回寫或改變模型節點、指紋、正規化參數及投票結果。
5. `API BIAS` 只允許調整決策後的類別偏移，不會也不得改寫原始指紋或樹節點。
6. 若模型檔案雜湊與 `MODEL_DATABASE.json` 記錄不一致，應停止使用模型並回報完整性錯誤，不得自動產生替代資料。

簡化邊界如下：

```text
MPU6050 實測資料 → 唯讀指紋／21 棵決策樹 → 5 秒活動狀態 → 馬達或其他設備控制
                         禁止修改或造假                 只讀取結果
```

給 AI 工具的同一份開發注意事項也記錄在 [`AGENTS.md`](AGENTS.md)。

更多資料：

- [決策樹、執行邏輯與步驟](DECISION_LOGIC.md)
- [四種活動指紋清單](FINGERPRINTS.md)
- [Serial API](API.md)
