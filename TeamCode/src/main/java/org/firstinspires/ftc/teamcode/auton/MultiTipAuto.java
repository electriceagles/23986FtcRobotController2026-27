package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.json.JSONArray;
import org.json.JSONObject;
import java.io.InputStream;
import java.io.ByteArrayOutputStream;
import static com.pedropathing.api.Paths.line;

/** Two/three actual hive-tip attempts. Read MULTITIP_SETUP.md before enabling. */
public abstract class MultiTipAuto extends OpMode {
    protected abstract boolean red();
    protected abstract int requestedTips();
    private enum Stage { ROUTE, SHOOT, SETTLE, DONE, ABORT }
    private Stage stage = Stage.ABORT;
    private String status = "Not configured";
    private JSONObject config;
    private JSONArray cycles, route;
    private Follower follower;
    private DcMotor rotate, shooter, intake1, intake2;
    private Limelight3A camera;
    private final ElapsedTime match = new ElapsedTime(), phase = new ElapsedTime(),
            segment = new ElapsedTime(), alignedTime = new ElapsedTime(), candidateTime = new ElapsedTime();
    private Pose previous;
    private int cycle, waypoint, origin, observedTips, candidate = -1, candidateFrames;
    private boolean ready, collecting, parking, pathStarted, sawExpected, fed;
    private double lastFrame = Double.NaN, error = Double.NaN;
    private int expectedState() throws Exception { return (config.getInt("initialState") + cycle) % 2; }
    private double number(String key) throws Exception {
        double value = config.getDouble(key);
        if (!Double.isFinite(value)) throw new IllegalArgumentException(key + " must be finite");
        return value;
    }
    private Pose pose(JSONArray a) throws Exception {
        if (a.length() != 3) throw new IllegalArgumentException("Pose needs x,y,heading");
        double x=a.getDouble(0), y=a.getDouble(1), h=a.getDouble(2);
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(h)
                || x < 0 || x > 144 || y < 0 || y > 144) throw new IllegalArgumentException("Invalid pose");
        return PoseFactory.degrees().of(x,y,h);
    }
    private void validateRoute(JSONArray a, boolean allowEmpty) throws Exception {
        if (!allowEmpty && a.length()==0) throw new IllegalArgumentException("Measured routes are required");
        for (int i=0;i<a.length();i++) pose(a.getJSONArray(i));
    }
    @Override public void init() {
        Scheduler.reset();
        try {
            String name = red() ? "multitip-red.json" : "multitip-blue.json";
            try (InputStream in = hardwareMap.appContext.getAssets().open(name);
                 ByteArrayOutputStream out = new ByteArrayOutputStream()) {
                byte[] buffer = new byte[4096]; int n;
                while ((n=in.read(buffer))!=-1) out.write(buffer,0,n);
                config = new JSONObject(out.toString("UTF-8"));
            }
            if (!config.getBoolean("configured")) throw new IllegalArgumentException("Fill measured routes, tag groups and turret settings in " + name);
            if (config.getInt("initialState") < 0 || config.getInt("initialState") > 1) throw new IllegalArgumentException("initialState must be 0 or 1");
            JSONArray groups=config.getJSONArray("stateTagIds");
            if (groups.length()!=2) throw new IllegalArgumentException("Two distinct stable-state tag groups required");
            java.util.HashSet<Integer> ids=new java.util.HashSet<>();
            for(int state=0;state<2;state++) {
                JSONArray group=groups.getJSONArray(state);
                if(group.length()==0) throw new IllegalArgumentException("Empty tag group");
                for(int j=0;j<group.length();j++) {
                    int id=group.getInt(j);
                    if(id<0 || !ids.add(id)) throw new IllegalArgumentException("Tag IDs must be nonnegative and unique");
                }
            }
            if (config.getJSONArray("aimOffsetDegrees").length()!=2) throw new IllegalArgumentException("Two aim offsets required");
            for(int i=0;i<2;i++) if(!Double.isFinite(config.getJSONArray("aimOffsetDegrees").getDouble(i))) throw new IllegalArgumentException("Invalid aim offset");
            for(String key:new String[]{"shotTimeoutSeconds","segmentTimeoutSeconds","tipSettleSeconds","spinupSeconds","alignmentSeconds","tipConfirmSeconds","aimKp","maxRotatePower","toleranceDegrees"})
                if(number(key)<=0) throw new IllegalArgumentException(key+" must be positive");
            if(number("maxRotatePower")>1 || Math.abs(number("rotateSign"))!=1) throw new IllegalArgumentException("Invalid rotation settings");
            if(config.getInt("pipeline")<0 || config.getInt("minRotateTicks")>=config.getInt("maxRotateTicks")) throw new IllegalArgumentException("Configure pipeline and turret limits");
            if(number("deadlineSeconds")<=0 || number("deadlineSeconds")>29.5) throw new IllegalArgumentException("Deadline must be 0..29.5 seconds");
            cycles=config.getJSONArray("cycles");
            if(cycles.length()<requestedTips()) throw new IllegalArgumentException("Not enough cycle routes");
            for(int i=0;i<requestedTips();i++) {
                JSONObject c=cycles.getJSONObject(i);
                validateRoute(c.getJSONArray("collectRoute"),i==0);
                validateRoute(c.getJSONArray("shootRoute"),false);
            }
            validateRoute(config.getJSONArray("parkAfter"+requestedTips()),false);
            previous=pose(config.getJSONArray("start"));
            follower=Constants.create(hardwareMap);
            if(follower==null) throw new IllegalStateException("Constants.create() returns null: configure Pedro drivetrain/localizer first");
            follower.setPose(previous);
            rotate=hardwareMap.get(DcMotor.class,"rotate");
            shooter=hardwareMap.get(DcMotor.class,"shooter");
            intake1=hardwareMap.get(DcMotor.class,"intake1");
            intake2=hardwareMap.get(DcMotor.class,"intake2");
            stopMechanisms();
            rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            origin=rotate.getCurrentPosition();
            camera=hardwareMap.get(Limelight3A.class,"limelight");
            camera.pipelineSwitch(config.getInt("pipeline")); camera.start();
            ready=true; status="Ready: "+requestedTips()+" tip attempts";
        } catch(Exception e) { abort(e.getMessage()); }
        telemetry.addData("Setup",status); telemetry.update();
    }
    @Override public void init_loop() { telemetry.addData("Setup",status); telemetry.update(); }
    @Override public void start() {
        if(!ready) return;
        cycle=0; observedTips=0; match.reset(); parking=false;
        Scheduler.schedule(Command.build()
                .setStart(() -> { try { beginCycle(); } catch(Exception e) { abort(e.getMessage()); } })
                .setExecute(() -> { try { tick(); } catch(Exception e) { abort(e.getMessage()); } })
                .setDone(() -> stage==Stage.DONE || stage==Stage.ABORT)
                .setEnd(reason -> { stopMechanisms(); if(follower!=null) follower.stop(); })
                .requiring(follower,rotate,shooter,intake1,intake2));
    }
    private void beginRoute(JSONArray points, boolean intake) {
        route=points; collecting=intake; waypoint=0; pathStarted=false;
        stage=Stage.ROUTE; status=parking ? "Parking" : (intake ? "Collecting" : "Driving to shoot");
        stopMechanisms();
    }
    private void beginCycle() throws Exception {
        JSONObject c=cycles.getJSONObject(cycle);
        if(c.getJSONArray("collectRoute").length()>0) beginRoute(c.getJSONArray("collectRoute"),true);
        else beginRoute(c.getJSONArray("shootRoute"),false);
    }
    private void tick() throws Exception {
        if(stage==Stage.ABORT || stage==Stage.DONE) return;
        if(match.seconds()>=number("deadlineSeconds")) { abort("Autonomous deadline reached"); return; }
        switch(stage) {
            case ROUTE:
                if(waypoint>=route.length()) {
                    intake1.setPower(0); intake2.setPower(0);
                    if(parking) { stage=Stage.DONE; status="Park complete"; return; }
                    if(collecting) { beginRoute(cycles.getJSONObject(cycle).getJSONArray("shootRoute"),false); return; }
                    stage=Stage.SHOOT; status="Aiming for tip "+(cycle+1);
                    phase.reset(); alignedTime.reset(); candidate=-1; candidateFrames=0;
                    sawExpected=false; fed=false; lastFrame=Double.NaN; shooter.setPower(1); return;
                }
                if(!pathStarted) {
                    Pose next=pose(route.getJSONArray(waypoint));
                    follower.follow(line(previous,next).linear(previous,next));
                    segment.reset(); pathStarted=true;
                    intake1.setPower(collecting ? 1 : 0); intake2.setPower(collecting ? 1 : 0);
                } else if(segment.seconds()>number("segmentTimeoutSeconds")) abort("Drive segment timeout");
                else if(!follower.isBusy()) {
                    if(follower.distanceToEndpoint()>2) { abort("Drive ended outside endpoint tolerance"); return; }
                    previous=pose(route.getJSONArray(waypoint)); waypoint++; pathStarted=false;
                }
                break;
            case SHOOT: aimAndObserveTip(); break;
            case SETTLE:
                if(phase.seconds()>=number("tipSettleSeconds")) {
                    cycle++;
                    if(cycle>=requestedTips()) { parking=true; beginRoute(config.getJSONArray("parkAfter"+requestedTips()),false); }
                    else beginCycle();
                }
                break;
            default: break;
        }
    }
    private void aimAndObserveTip() throws Exception {
        if(phase.seconds()>number("shotTimeoutSeconds")) { abort("No confirmed hive-state change; not counting a tip"); return; }
        LLResult frame=camera.getLatestResult();
        int visible=-1; error=Double.NaN;
        boolean fresh=frame!=null && frame.isValid() && frame.getStaleness()<=150
                && frame.getPipelineIndex()==config.getInt("pipeline");
        if(fresh) {
            for(LLResultTypes.FiducialResult tag:frame.getFiducialResults()) {
                for(int s=0;s<2;s++) {
                    JSONArray ids=config.getJSONArray("stateTagIds").getJSONArray(s);
                    for(int i=0;i<ids.length();i++) if(ids.getInt(i)==tag.getFiducialId()) {
                        if(visible==-1) visible=s; else if(visible!=s) visible=-2;
                        if(s==expectedState() && !Double.isFinite(error)) error=tag.getTargetXDegrees()-config.getJSONArray("aimOffsetDegrees").getDouble(s);
                    }
                }
            }
        }
        if(visible!=expectedState()) error=Double.NaN;
        if(visible<0) { candidate=-1; candidateFrames=0; }
        if(fresh && frame.getTimestamp()!=lastFrame) {
            lastFrame=frame.getTimestamp();
            if(visible>=0) {
                if(candidate!=visible) { candidate=visible; candidateFrames=1; candidateTime.reset(); }
                else candidateFrames++;
                if(candidateFrames>=3 && candidateTime.seconds()>=number("tipConfirmSeconds")) {
                    if(visible==expectedState()) sawExpected=true;
                    else if(sawExpected && fed) {
                        observedTips++; stopMechanisms(); stage=Stage.SETTLE; phase.reset();
                        status="Observed tip "+observedTips+"; waiting for balls"; return;
                    }
                }
            }
        }
        boolean aligned=Double.isFinite(error) && Math.abs(error)<=number("toleranceDegrees");
        if(!aligned) alignedTime.reset();
        double power=Double.isFinite(error) && !aligned ? Range.clip(number("rotateSign")*number("aimKp")*error,-number("maxRotatePower"),number("maxRotatePower")) : 0;
        int ticks=rotate.getCurrentPosition()-origin;
        if((ticks<=config.getInt("minRotateTicks") && power<0) || (ticks>=config.getInt("maxRotateTicks") && power>0)) power=0;
        rotate.setPower(power);
        boolean feed=sawExpected && aligned && alignedTime.seconds()>=number("alignmentSeconds") && phase.seconds()>=number("spinupSeconds");
        intake1.setPower(feed ? 1 : 0); intake2.setPower(feed ? 1 : 0);
        fed |= feed;
    }
    private void stopMechanisms() {
        for(DcMotor m:new DcMotor[]{rotate,shooter,intake1,intake2}) if(m!=null) m.setPower(0);
    }
    private void abort(String reason) {
        stage=Stage.ABORT; status=reason; ready=false; stopMechanisms(); if(follower!=null) follower.stop();
    }
    @Override public void loop() {
        if(ready && follower!=null) follower.update();
        Scheduler.execute();
        telemetry.addData("Status",status); telemetry.addData("Stage",stage);
        telemetry.addData("Tips inferred from vision",observedTips+" / "+requestedTips());
        telemetry.addData("Aim error",error); telemetry.update();
    }
    @Override public void stop() {
        Scheduler.reset(); stopMechanisms(); if(follower!=null) follower.stop(); if(camera!=null) camera.stop();
    }
}
