#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ctype.h>

ESP8266WebServer server(80);

/* ── Live sensor globals ─────────────────────────────────────────────────── */
int T=0,H=0,S_code=0,F_val=0,IR=0,DT=0,DS=0,MT=0,MS=0,XS=0,RADAR=0;
int AX=0,AY=0,AZ=0,MOT=0,TILT=0,SHOCK=0,VIB=0;
unsigned long TS=0;
String AQ_STR="GOOD";
String bufLine="";

/* ── Radar presence log (last 20 events) ─────────────────────────────────── */
struct RadarEvent { unsigned long t; int state; }; /* state: 1=detected, 0=cleared */
RadarEvent radarLog[20];
int radarLogCount = 0;
int prevRADAR     = -1;   /* -1 = uninitialised, tracks edge detection */

/* ── Motion log (last 8 events) ─────────────────────────────────────────── */
struct MotionLogEntry { unsigned long t; int e; };
MotionLogEntry motionLog[8];
int motionLogCount=0;

/* ═══════════════════════════════════════════════════════════════════════════
   CSV circular log  – 200 rows
   ═══════════════════════════════════════════════════════════════════════════ */
#define CSV_LOG_SIZE 200

struct LogRow {
    unsigned long ts;
    int8_t   T, H;
    uint8_t  S_code, flame;
    int16_t  IR;
    int16_t  AX, AY, AZ;
    char     AQ[5];
    uint8_t  tilt_active, shock_active, vib_active, mot;
    uint16_t tilt_count, shock_count, vib_count;
    uint8_t  radar;   /* NEW: 1 = presence detected, 0 = clear */
};

static LogRow csvBuf[CSV_LOG_SIZE];
static int    csvHead  = 0;
static int    csvCount = 0;

static void csvAppend(){
    int idx = (csvHead + csvCount) % CSV_LOG_SIZE;
    if(csvCount == CSV_LOG_SIZE) csvHead = (csvHead + 1) % CSV_LOG_SIZE;
    else csvCount++;
    LogRow &r = csvBuf[idx];
    r.ts           = TS;
    r.T            = (int8_t)T;
    r.H            = (int8_t)H;
    r.S_code       = (uint8_t)S_code;
    r.flame        = (uint8_t)(F_val ? 1 : 0);
    r.IR           = (int16_t)IR;
    r.AX           = (int16_t)AX;
    r.AY           = (int16_t)AY;
    r.AZ           = (int16_t)AZ;
    strncpy(r.AQ, AQ_STR.c_str(), 4); r.AQ[4] = '\0';
    r.tilt_active  = (uint8_t)((MOT & 1) ? 1 : 0);
    r.shock_active = (uint8_t)((MOT & 2) ? 1 : 0);
    r.vib_active   = (uint8_t)((MOT & 4) ? 1 : 0);
    r.mot          = (uint8_t)MOT;
    r.tilt_count   = (uint16_t)TILT;
    r.shock_count  = (uint16_t)SHOCK;
    r.vib_count    = (uint16_t)VIB;
    r.radar        = (uint8_t)(RADAR ? 1 : 0);   /* NEW */
}

/* ── JSON parsing helpers ─────────────────────────────────────────────────── */
static int parseJsonInt(const String &json, const char *key){
    String search="\""; search+=key; search+="\":";
    int idx=json.indexOf(search); if(idx<0) return 0;
    idx+=search.length();
    while(idx<(int)json.length() && json[idx]==' ') idx++;
    String num="";
    if(idx<(int)json.length() && json[idx]=='-'){ num+='-'; idx++; }
    while(idx<(int)json.length() && isDigit(json[idx])) num+=json[idx++];
    return num.toInt();
}
static String parseJsonStr(const String &json, const char *key){
    String search="\""; search+=key; search+="\":\"";
    int idx=json.indexOf(search); if(idx<0) return "";
    idx+=search.length(); String val="";
    while(idx<(int)json.length() && json[idx] != '"') val+=json[idx++];
    return val;
}
static void parseMotionLog(const String &json){
    motionLogCount=0;
    int keyIdx=json.indexOf("\"MLOG\":"); if(keyIdx<0) return;
    int arrStart=json.indexOf('[',keyIdx), arrEnd=json.indexOf(']',arrStart);
    if(arrStart<0||arrEnd<0||arrEnd<=arrStart) return;
    String arr=json.substring(arrStart+1,arrEnd); int pos=0;
    while(pos<(int)arr.length() && motionLogCount<8){
        int objStart=arr.indexOf('{',pos); if(objStart<0) break;
        int objEnd=arr.indexOf('}',objStart); if(objEnd<0) break;
        String obj=arr.substring(objStart,objEnd+1);
        motionLog[motionLogCount].t=(unsigned long)parseJsonInt(obj,"t");
        motionLog[motionLogCount].e=parseJsonInt(obj,"e");
        motionLogCount++; pos=objEnd+1;
    }
}
static const char* motionTypeName(int e){
    if(e&1) return "TILT"; if(e&2) return "SHOCK"; if(e&4) return "VIB"; return "UNKNOWN";
}

/*
 * Log a radar edge event (HIGH or LOW transition).
 * Called whenever RADAR changes state so the log shows arrivals and departures.
 */
static void addRadarEvent(int state){
    if(radarLogCount < 20){
        radarLog[radarLogCount].t = TS;
        radarLog[radarLogCount].state = state;
        radarLogCount++;
    } else {
        for(int i=0;i<19;i++) radarLog[i]=radarLog[i+1];
        radarLog[19].t = TS; radarLog[19].state = state;
    }
}

static void parseLine(const String &line){
    if(line.indexOf('{')<0) return;
    T=parseJsonInt(line,"T"); H=parseJsonInt(line,"H"); S_code=parseJsonInt(line,"S");
    F_val=parseJsonInt(line,"F"); IR=parseJsonInt(line,"IR"); DT=parseJsonInt(line,"DT");
    DS=parseJsonInt(line,"DS"); MT=parseJsonInt(line,"MT"); MS=parseJsonInt(line,"MS");
    XS=parseJsonInt(line,"XS"); RADAR=parseJsonInt(line,"RADAR");
    AX=parseJsonInt(line,"AX"); AY=parseJsonInt(line,"AY"); AZ=parseJsonInt(line,"AZ");
    MOT=parseJsonInt(line,"MOT"); TILT=parseJsonInt(line,"TILT");
    SHOCK=parseJsonInt(line,"SHOCK"); VIB=parseJsonInt(line,"VIB");
    TS=(unsigned long)parseJsonInt(line,"ts");
    AQ_STR=parseJsonStr(line,"AQ"); if(AQ_STR.length()==0) AQ_STR="GOOD";
    parseMotionLog(line);

    /* Detect radar state change and log the edge */
    if(prevRADAR != RADAR){
        addRadarEvent(RADAR);
        prevRADAR = RADAR;
    }
}

/* ── Display helpers ──────────────────────────────────────────────────────── */
static String mgToG(int val){ if(val<0) val=-val; return String(val/1000.0f,3); }
static String irDisplay(){ if(IR<-3000) return "--"; return String(IR/10.0f,1); }

/* ── Motion detection (ESP-side) ─────────────────────────────────────────── */
#define TILT_THRESH_MG   300
#define VIB_THRESH_MG    120
#define SHOCK_THRESH_MG  600

static int  prevAX=0, prevAY=0, prevAZ=0;
static bool motionReady=false;

static void addMotionLogEntry(int eventBits){
    if(motionLogCount<8){
        motionLog[motionLogCount].t=TS; motionLog[motionLogCount].e=eventBits; motionLogCount++;
    } else {
        for(int i=0;i<7;i++) motionLog[i]=motionLog[i+1];
        motionLog[7].t=TS; motionLog[7].e=eventBits;
    }
}
static void computeMotion(){
    int dX=AX-prevAX,dY=AY-prevAY,dZ=AZ-prevAZ;
    if(dX<0)dX=-dX;if(dY<0)dY=-dY;if(dZ<0)dZ=-dZ;
    int maxDelta=dX>dY?dX:dY;if(dZ>maxDelta)maxDelta=dZ;
    int absAX=AX<0?-AX:AX,absAY=AY<0?-AY:AY;
    bool isTilt=(absAX>TILT_THRESH_MG||absAY>TILT_THRESH_MG);
    bool isShock=motionReady&&(maxDelta>SHOCK_THRESH_MG);
    bool isVib=motionReady&&!isShock&&(maxDelta>VIB_THRESH_MG);
    int newMOT=0,logBits=0;
    if(isTilt){newMOT|=1;if(!(MOT&1)){TILT++;logBits|=1;}}
    if(isShock){newMOT|=2;if(!(MOT&2)){SHOCK++;logBits|=2;}}
    if(isVib){newMOT|=4;if(!(MOT&4)){VIB++;logBits|=4;}}
    if(logBits) addMotionLogEntry(logBits);
    MOT=newMOT; prevAX=AX;prevAY=AY;prevAZ=AZ; motionReady=true;
}

/* ── Smoke / badge helpers ────────────────────────────────────────────────── */
static String smokeLabel(){ if(S_code==0)return "NORMAL";if(S_code==1)return "WARNING";return "DANGER";}
static String smokeColor(){ if(S_code==0)return "#27ae60";if(S_code==1)return "#f39c12";return "#e74c3c";}
static String motionBadge(bool active,const String &onColor){
    return String("<span class='badge' style='color:")+(active?onColor:"#27ae60")+"'>"+(active?"DETECTED":"NONE")+"</span>";
}
static String motionLogHtml(){
    String s="";
    if(motionLogCount==0){
        s+="<div class='card wide'><div class='card-label'>Recent Motion Events</div>"
           "<div class='feat-value' style='font-size:16px;color:#8b949e'>No events yet</div></div>";
        return s;
    }
    s+="<div class='card wide'><div class='card-label'>Recent Motion Events</div>";
    for(int i=motionLogCount-1;i>=0;i--){
        s+="<div style='margin-top:6px;font-family:monospace;font-size:14px;color:#64b5f6;'>";
        s+=String(motionLog[i].t);s+="s &mdash; ";s+=motionTypeName(motionLog[i].e);
        s+=" (bits=";s+=String(motionLog[i].e);s+=")</div>";
    }
    s+="</div>";return s;
}

/*
 * Human presence log HTML – shows last 20 radar edge events
 * in reverse chronological order (most recent at top).
 */
static String radarLogHtml(){
    String s="<div class='card wide'><div class='card-label'>Human Presence Log (last 20 events)</div>";
    if(radarLogCount==0){
        s+="<div class='feat-value' style='font-size:16px;color:#8b949e'>No events yet</div></div>";
        return s;
    }
    for(int i=radarLogCount-1;i>=0;i--){
        const char *label = radarLog[i].state ? "DETECTED" : "CLEARED";
        const char *color = radarLog[i].state ? "#e74c3c"  : "#27ae60";
        s+="<div style='margin-top:6px;font-family:monospace;font-size:14px;'>";
        s+="<span style='color:#8b949e'>"+String(radarLog[i].t)+"s</span>";
        s+=" &mdash; <span style='color:"+String(color)+";font-weight:700'>"+String(label)+"</span>";
        s+="</div>";
    }
    s+="</div>";return s;
}

/* ═══════════════════════════════════════════════════════════════════════════
   HTTP handlers
   ═══════════════════════════════════════════════════════════════════════════ */

void handleRoot(){
    String logStatus = String(csvCount) + " / " + String(CSV_LOG_SIZE) + " rows";
    String radarColor  = RADAR ? "#e74c3c" : "#27ae60";
    String radarLabel  = RADAR ? "DETECTED" : "CLEAR";

    String html = R"HTML(<!DOCTYPE html><html><head>
<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>K66 Sensor Dashboard</title>
<style>
body{font-family:Arial,sans-serif;background:#0d1117;color:#e6edf3;margin:0;padding:20px}
.wrap{max-width:1100px;margin:0 auto}
h1{margin:0 0 4px 0;font-size:24px}
.sub{color:#8b949e;margin-bottom:20px;font-size:14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:14px}
.card{background:#161b22;border:1px solid #30363d;border-radius:14px;padding:16px}
.wide{grid-column:span 2}
.card-label{font-size:13px;color:#8b949e;margin-bottom:8px}
.card-value{font-size:28px;font-weight:700}
.badge{font-size:20px;font-weight:700}
.section-hdr{margin:22px 0 10px;font-size:15px;font-weight:700;color:#58a6ff;
             border-bottom:1px solid #21262d;padding-bottom:4px}
.feat-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px}
.feat-card{background:#161b22;border:1px solid #30363d;border-radius:14px;padding:14px}
.feat-label{font-size:12px;color:#8b949e}
.feat-value{font-size:22px;font-weight:700;margin-top:6px}
.log-bar{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin:6px 0 10px}
.btn{display:inline-block;padding:9px 20px;border-radius:9px;font-size:14px;font-weight:700;
     text-decoration:none;border:none;cursor:pointer}
.btn-dl{background:#238636;color:#fff}.btn-dl:hover{background:#2ea043}
.btn-cl{background:#6e1a1a;color:#fff}.btn-cl:hover{background:#b22222}
.log-info{color:#8b949e;font-size:13px}
.radar-pulse{animation:pulse 1.2s infinite}
@keyframes pulse{0%{opacity:1}50%{opacity:0.4}100%{opacity:1}}
</style>
<meta http-equiv='refresh' content='2'>
</head><body><div class='wrap'>
<h1>K66 Sensor + Motion + Radar Dashboard</h1>
<div class='sub'>ESP8266 Access Point &mdash; live UART JSON feed</div>

<div class='section-hdr'>&#9658; HUMAN PRESENCE (RCWL-0516)</div>
<div class='grid'>
<div class='card' style='border-color:)HTML" + radarColor + R"HTML('>
  <div class='card-label'>Radar Detection</div>
  <div class='card-value )HTML" + String(RADAR?"radar-pulse":"") + R"HTML(' style='color:)HTML" + radarColor + R"HTML('>
    )HTML" + radarLabel + R"HTML(
  </div>
</div>
</div>

<div class='section-hdr'>&#9658; CORE SENSORS</div>
<div class='grid'>
<div class='card'><div class='card-label'>Temperature</div>
  <div class='card-value'>)HTML" + String(T) + R"HTML( &#176;C</div></div>
<div class='card'><div class='card-label'>Humidity</div>
  <div class='card-value'>)HTML" + String(H) + R"HTML( %</div></div>
<div class='card'><div class='card-label'>Smoke</div>
  <div class='card-value' style='color:)HTML" + smokeColor() + R"HTML('>)HTML" + smokeLabel() + R"HTML(</div></div>
<div class='card'><div class='card-label'>Flame</div>
  <div class='card-value' style='color:)HTML" + String(F_val?"#e74c3c":"#27ae60") + R"HTML('>)HTML" + String(F_val?"YES":"NO") + R"HTML(</div></div>
<div class='card'><div class='card-label'>IR Temp</div>
  <div class='card-value'>)HTML" + irDisplay() + R"HTML( &#176;C</div></div>
<div class='card'><div class='card-label'>Air Quality</div>
  <div class='card-value'>)HTML" + AQ_STR + R"HTML(</div></div>
</div>

<div class='section-hdr'>&#9658; MOTION STATUS</div>
<div class='grid'>
<div class='card'><div class='card-label'>Tilt Active</div>
  <div style='margin-top:4px'>)HTML" + motionBadge(MOT&1,"#f39c12") + R"HTML(</div></div>
<div class='card'><div class='card-label'>Shock Active</div>
  <div style='margin-top:4px'>)HTML" + motionBadge(MOT&2,"#e74c3c") + R"HTML(</div></div>
<div class='card'><div class='card-label'>Vibration Active</div>
  <div style='margin-top:4px'>)HTML" + motionBadge(MOT&4,"#3498db") + R"HTML(</div></div>
<div class='card'><div class='card-label'>Motion Bitmask</div>
  <div class='card-value'>)HTML" + String(MOT) + R"HTML(</div></div>
</div>

<div class='section-hdr'>&#9658; ACCELEROMETER</div>
<div class='feat-grid'>
<div class='feat-card'><div class='feat-label'>AX</div>
  <div class='feat-value'>)HTML" + mgToG(AX) + R"HTML( g</div></div>
<div class='feat-card'><div class='feat-label'>AY</div>
  <div class='feat-value'>)HTML" + mgToG(AY) + R"HTML( g</div></div>
<div class='feat-card'><div class='feat-label'>AZ</div>
  <div class='feat-value'>)HTML" + mgToG(AZ) + R"HTML( g</div></div>
<div class='feat-card'><div class='feat-label'>Tilt Count</div>
  <div class='feat-value'>)HTML" + String(TILT) + R"HTML(</div></div>
<div class='feat-card'><div class='feat-label'>Shock Count</div>
  <div class='feat-value'>)HTML" + String(SHOCK) + R"HTML(</div></div>
<div class='feat-card'><div class='feat-label'>Vibration Count</div>
  <div class='feat-value'>)HTML" + String(VIB) + R"HTML(</div></div>
</div>

<div class='section-hdr'>&#9658; UPTIME</div>
<div class='grid'>
<div class='card'><div class='card-label'>Uptime</div>
  <div class='card-value'>)HTML" + String((unsigned long)TS) + R"HTML( s</div></div>
</div>

<div class='section-hdr'>&#9658; CSV DATA LOG</div>
<div class='log-bar'>
  <a class='btn btn-dl' href='/log.csv' download='k66_log.csv'>&#11015; Download CSV</a>
  <a class='btn btn-cl' href='/clearlog'
     onclick="return confirm('Clear all )HTML" + String(csvCount) + R"HTML( logged rows?')">
    &#128465; Clear Log</a>
  <span class='log-info'>)HTML" + logStatus + R"HTML( &mdash; circular buffer</span>
</div>

<div class='section-hdr'>&#9658; HUMAN PRESENCE LOG</div>
<div class='grid'>)HTML" + radarLogHtml() + R"HTML(</div>

<div class='section-hdr'>&#9658; RECENT MOTION EVENTS</div>
<div class='grid'>)HTML" + motionLogHtml() + R"HTML(</div>

</div></body></html>)HTML";

    server.sendHeader("Cache-Control","no-store");
    server.send(200,"text/html",html);
}

/* ── /log.csv ────────────────────────────────────────────────────────────── */
void handleCSV(){
    server.sendHeader("Content-Disposition","attachment; filename=\"k66_log.csv\"");
    server.sendHeader("Cache-Control","no-store");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200,"text/csv","");

    /* Header row – radar column added at the end */
    server.sendContent(
        "uptime_s,"
        "temp_C,humidity_pct,"
        "smoke_code,smoke_label,"
        "flame,"
        "ir_temp_C,"
        "ax_mg,ay_mg,az_mg,"
        "air_quality,"
        "tilt_active,shock_active,vib_active,"
        "motion_bitmask,"
        "tilt_count,shock_count,vib_count,"
        "radar_presence\r\n"          /* NEW column */
    );

    for(int i=0;i<csvCount;i++){
        const LogRow &r=csvBuf[(csvHead+i)%CSV_LOG_SIZE];
        const char *smokeStr=(r.S_code==0)?"NORMAL":(r.S_code==1)?"WARNING":"DANGER";
        char irBuf[10];
        if(r.IR<-3000) snprintf(irBuf,sizeof(irBuf),"--");
        else           snprintf(irBuf,sizeof(irBuf),"%.1f",r.IR/10.0f);
        char row[180];
        snprintf(row,sizeof(row),
            "%lu,"
            "%d,%d,"
            "%u,%s,"
            "%u,"
            "%s,"
            "%d,%d,%d,"
            "%s,"
            "%u,%u,%u,"
            "%u,"
            "%u,%u,%u,"
            "%u\r\n",           /* NEW: radar */
            r.ts,
            (int)r.T,(int)r.H,
            (unsigned)r.S_code,smokeStr,
            (unsigned)r.flame,
            irBuf,
            (int)r.AX,(int)r.AY,(int)r.AZ,
            r.AQ,
            (unsigned)r.tilt_active,(unsigned)r.shock_active,(unsigned)r.vib_active,
            (unsigned)r.mot,
            (unsigned)r.tilt_count,(unsigned)r.shock_count,(unsigned)r.vib_count,
            (unsigned)r.radar       /* NEW */
        );
        server.sendContent(row);
        yield();
    }
    server.sendContent("");
}

/* ── /clearlog ───────────────────────────────────────────────────────────── */
void handleClearLog(){
    csvHead=0;csvCount=0;radarLogCount=0;prevRADAR=-1;
    server.sendHeader("Location","/");server.send(303,"text/plain","Redirecting");
}

/* ── /api ────────────────────────────────────────────────────────────────── */
void handleAPI(){
    String json="{\"T\":";json+=T;json+=",\"H\":";json+=H;
    json+=",\"S\":";json+=S_code;json+=",\"F\":";json+=F_val;
    json+=",\"IR\":";json+=IR;json+=",\"AQ\":\"";json+=AQ_STR;
    json+="\",\"AX\":";json+=AX;json+=",\"AY\":";json+=AY;json+=",\"AZ\":";json+=AZ;
    json+=",\"RADAR\":";json+=RADAR;   /* NEW */
    json+=",\"MOT\":";json+=MOT;json+=",\"TILT\":";json+=TILT;
    json+=",\"SHOCK\":";json+=SHOCK;json+=",\"VIB\":";json+=VIB;
    json+=",\"MLOG\":[";
    for(int i=0;i<motionLogCount;i++){
        if(i>0)json+=",";
        json+="{\"t\":";json+=motionLog[i].t;json+=",\"e\":";json+=motionLog[i].e;json+="}";
    }
    json+="],\"ts\":";json+=(unsigned long)TS;json+="}";
    server.sendHeader("Cache-Control","no-store");
    server.sendHeader("Access-Control-Allow-Origin","*");
    server.send(200,"application/json",json);
}

/* ═══════════════════════════════════════════════════════════════════════════
   Setup / Loop
   ═══════════════════════════════════════════════════════════════════════════ */
void setup(){
    Serial.begin(9600);Serial.swap();
    WiFi.mode(WIFI_AP);WiFi.softAP("K66_SENSORS","12345678");
    server.on("/",         handleRoot);
    server.on("/api",      handleAPI);
    server.on("/log.csv",  handleCSV);
    server.on("/clearlog", handleClearLog);
    server.begin();
}

void loop(){
    server.handleClient();
    while(Serial.available()){
        char c=(char)Serial.read();
        if(c=='\n'||c=='\r'){
            if(bufLine.length()>0){
                bufLine.trim();
                parseLine(bufLine);
                computeMotion();
                csvAppend();
                bufLine="";
            }
        } else {
            if(bufLine.length()<600) bufLine+=c;
        }
    }
}