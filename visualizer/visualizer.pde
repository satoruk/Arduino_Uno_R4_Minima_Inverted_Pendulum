import processing.serial.*;

Serial myPort;
String arduinoPort = "/dev/cu.usbmodem1201";
float[]data = new float[12];
PGraphics pg;


// boolean headerAdded = false; // ヘッダーが追加されたかどうか
int lastLogTime = 0; // 最後にログを書き込んだ時間

void keyPressed() {
  // `keyPressed` でプログラム終了時にファイルを閉じる
  if (key == 'q') {
    println("終了します");
    exit();
  }
}

void setup() {
  size(600, 400, P3D);
  
  pg = createGraphics(1200, 800, P3D);
  pg.smooth();
  pg.lights();
  
  myPort = new Serial(this, arduinoPort, 115200);
}

void draw() {
  pg.beginDraw();
  pg.strokeWeight(1);
  pg.stroke(0, 0, 0);
  pg.background(230);
  pg.translate(pg.width / 2, pg.height / 2, 0);
  
  // 姿勢に応じて回転
  pg.rotateY(radians(data[11]));
  pg.rotateX(radians( -data[10]));
  pg.rotateZ(radians( -data[9]));
  
  int size = 20;
  pg.fill(150, 150, 255, 150); // 半透明（アルファ値 150）
  pg.box(20 * size, 1 * size, 10 * size);
  pg.translate(0, -4 * size, 0);
  
  drawAxes(100);
  
  pg.endDraw();
  
  image(pg, 0, 0, width, height);
  
  displayInfo();
}

void displayInfo() {
  fill(0); // 黒色で描画
  textSize(12);
  
  String[] labels = {
    "Acc X:", "Acc Y:", "Acc Z:",
    "Gyro X:", "Gyro Y:", "Gyro Z:",
    "Mag X:", "Mag Y:", "Mag Z:",
    "Roll:", "Pitch:", "Yaw:"
  };
  
  float xLabel = 10;
  float xValue = 100;
  float yStart = 10;
  float lineHeight = 20;
  
  for (int i = 0; i < labels.length; i++) {
    float y = yStart + i * lineHeight;
    
    // ラベルを右寄せで描画
    textAlign(LEFT, TOP);
    text(labels[i], xLabel, y);
    
    // 値を左寄せで描画
    textAlign(RIGHT, TOP);
    text(nf(data[i], 1, 2), xValue, y);
  }
}

void drawAxes(float length) {
  pg.strokeWeight(5);
  
  // X軸（赤）
  pg.stroke(255, 0, 0);
  pg.line(0, 0, 0, length, 0, 0);
  
  // Y軸（緑）
  pg.stroke(0, 255, 0);
  pg.line(0, 0, 0, 0, -length, 0);
  
  // Z軸（青）
  pg.stroke(0, 0, 255);
  pg.line(0, 0, 0, 0, 0, -length);
}

void serialEvent(Serial p) {
  String line = myPort.readStringUntil('\n');
  if (line == null) {
    return;
  }
  
  Float[] values = new Float[0];
  
  line = trim(line);
  if (!line.startsWith("V,")) {
    println("フォーマットが異なります: " + line);
    return;
  }
  
  String[] rawValues = line.split(",");
  try {
    for (int i = 1; i < rawValues.length; i++) {
      data[i - 1] = Float.parseFloat(rawValues[i]);
    }
  } catch(NumberFormatException e) {
    println("パースエラー: " + e.getMessage());
    return;
  }
  
  // data[0] = values[9].floatValue(); 
  // data[1] = values[10].floatValue(); 
  // data[2] = values[11].floatValue(); 
  
  // // デバッグ出力
  // if (millis() - lastLogTime >= 1000) {
  //   lastLogTime = millis();
  //   println("                    Acc,                   Gyro,                    Mag");
  //   println("      X,      Y,      Z,      X,      Y,      Z,      X,      Y,      Z,   Roll,  Pitch,    Yaw");
  // }
  
  // String format = "%7.2f";
  // String[] formats = new String[values.length];
  // for (int i = 0; i < formats.length; i++) {
  //   formats[i] = format;
  // }
  // String all_format = join(formats, ",");
  // String formatted = String.format(all_format, values);
  // println(formatted);
}
