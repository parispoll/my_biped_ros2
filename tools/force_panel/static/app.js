// Browser UI only: collect settings, submit allowed jobs and render saved results.
// Robot motion, PID and pass/fail audits live in the Python folders.
'use strict';
const $=id=>document.getElementById(id);
const token=document.querySelector('meta[name="force-panel-token"]').content;
let direction='forward',state={active_id:null,jobs:[]},selectedId=null,closing=false,submitting=false,lockedJobId=null,historyVersion=null,roadmapVersion=null,connectionFailed=false;
function notice(message){$('notice').textContent=message;$('notice').hidden=!message;}
// Display the meaning of the selected mode; this never commands a joint.
function preview(){
  const walkMode=['alternating_walk','alternating_walk_preview'].includes($('mode').value);
  $('walk-settings').hidden=!walkMode;
  const walkN=Number($('walk-steps').value),walkMm=Number($('walk-step-mm').value),walkScale=Number($('walk-time-scale').value);
  const stepMode=['first_step','first_step_preview','alternating_walk','alternating_walk_preview'].includes($('mode').value);
  if(stepMode){
    const offline=['first_step_preview','alternating_walk_preview'].includes($('mode').value);
    for(const id of ['lift-settings','lowering-settings','lowered-unload-settings','push-settings'])$(id).hidden=true;
    $('lift-mm').value='30';$('lowering-depth').value='40';$('lowered-shift-settings').hidden=false;
    $('lowered-shift-distance').hidden=true;document.querySelector('label[for="lowered-shift-distance"]').hidden=true;
    document.querySelector('label[for="lowered-shift-side"]').textContent='Supporting leg (opposite foot steps)';
    document.querySelector('#lowered-shift-settings .field-note').textContent='50 mm forward placement, 30 mm clearance. End with both feet loaded in a new staggered stance.';
    document.querySelector('.card.preview').hidden=true;$('gazebo').parentElement.hidden=offline;
    document.querySelector('h1').innerHTML=offline?'Plan the step.<br>Check the new stance.':'Lift. Step forward.<br>Land and share the load.';
    document.querySelector('.eyebrow').textContent=offline?'OFFLINE / FIRST FORWARD STEP':'EXPERIMENT / FIRST FORWARD STEP';
    document.querySelector('.intro-copy').textContent='One deliberate foot placement. The original supporting foot stays planted. This is the first step toward an alternating gait.';
    document.querySelector('.envelope .mini-label').textContent='FOOT ADVANCE';document.querySelector('.envelope strong').textContent='50 mm';
    document.querySelector('.envelope p').textContent='30 mm clearance · 40 mm crouch · no push';
    document.querySelector('.setup h2').textContent=offline?'Check the step geometry':'Run one measured step';
    document.querySelector('.setup .section-heading p').textContent='Requires a measured same-side 30 mm lift.';
    $('mode-note').textContent=offline?'Check joint limits, collision envelopes, static torque reserve, speed and the new landing geometry. No simulation.':'241 simulated seconds after readiness: transfer, lift 15 s, advance 20 s, lower 15 s, confirm landing, reload 64 s, hold 8 s. About 18–20 wall minutes.';
    $('run').firstElementChild.textContent=offline?'Generate step preflight':'Run one 50 mm step';
    document.querySelector('.run-note').textContent=offline?'Predictions only; measured stepping is a separate trial.':'Sole position, fresh contact loss, landing and final 50/50 load are audited. Gazebo closes after recording. No return to the old foot positions.';
    if(walkMode){document.querySelector('h1').innerHTML=offline?'Plan alternating steps.<br>Check every stance.':'Transfer. Step.<br>Alternate and repeat.';document.querySelector('.eyebrow').textContent=offline?'OFFLINE / ALTERNATING WALK':'EXPERIMENT / ALTERNATING WALK';document.querySelector('.intro-copy').textContent=`${walkN} alternating placements, ${walkMm} mm step length and 30 mm clearance. Support roles and new foot locations are retained.`;document.querySelector('.envelope .mini-label').textContent='ALTERNATING PLACEMENTS';document.querySelector('.envelope strong').textContent=walkN+' steps';document.querySelector('.setup h2').textContent=offline?'Check the complete walking path':`Run ${walkN} alternating steps`;document.querySelector('.setup .section-heading p').textContent='Requires a measured first step and full-path preflight.';document.querySelector('#lowered-shift-settings .field-note').textContent='Select the initial supporting leg. The other leg steps first, then the legs alternate.';$('mode-note').textContent=offline?'Check every support exchange, swing path, static torque, collision envelope and boundary. No simulation.':`${walkN} placements at ${walkMm} mm nominal length, movement timing scale ${walkScale}. Motor caps unchanged; landing confirmation and measurement holds retained. Offline preview reports the exact duration.`;$('run').firstElementChild.textContent=offline?'Generate walking preflight':`Run ${walkN} alternating steps`;document.querySelector('.run-note').textContent=offline?'Predicted gait only; measured execution remains separate.':`Stops after ${walkN} placements and stable double support. Every placement, contact transition and motor command is audited.`;}
    return;
  }
  const liftMm=Number($('lift-mm').value),liftMove=liftMm===30?15:10,liftTotal=233+2*liftMove+4;$('lift-settings').hidden=!['foot_lift','foot_lift_preview'].includes($('mode').value);
  const liftPreview=$('mode').value==='foot_lift_preview';
  if(liftPreview){
    document.querySelector('.card.preview').hidden=true;$('gazebo').parentElement.hidden=true;
    for(const id of ['lowering-settings','lowered-unload-settings','push-settings'])$(id).hidden=true;
    $('lowered-shift-settings').hidden=false;$('lowered-shift-distance').hidden=true;document.querySelector('label[for="lowered-shift-distance"]').hidden=true;
    document.querySelector('label[for="lowered-shift-side"]').textContent='Supporting side';
    document.querySelector('#lowered-shift-settings .field-note').textContent=`Select the supporting side. Preview only: the other foot is planned ${liftMm} mm up and down from the zero-load endpoint.`;
    document.querySelector('h1').innerHTML='Plan the lift.<br>Check the physics.';document.querySelector('.eyebrow').textContent='OFFLINE / FIRST FOOT LIFT';
    document.querySelector('.intro-copy').textContent='Checked joint geometry and single-support static torques, before adding live lift and landing control. No simulation starts.';
    document.querySelector('.envelope .mini-label').textContent='PLANNED CLEARANCE';document.querySelector('.envelope strong').textContent=liftMm+' mm';document.querySelector('.envelope p').textContent='A prediction is not a measured foot lift.';
    document.querySelector('.setup h2').textContent='Preview single support';document.querySelector('.setup .section-heading p').textContent='Requires a verified same-side zero-load baseline.';
    $('mode-note').textContent='Compute lift, short hold and lowering references. Check sole corners, support COM margin, joint limits, motor reserve and independent gravitational virtual work. Full-body collision and live balance checks remain next.';
    $('run').firstElementChild.textContent='Generate offline lift preview';document.querySelector('.run-note').textContent='Saves a prediction plot and numerical checks. No ROS motor commands, no Gazebo and no measured contact transition.';
    return;
  }
  document.querySelector('label[for="lowered-shift-side"]').textContent='Lowered transfer direction';
  const bench=$('mode').value==='contact_bench';
  document.querySelector('.card.preview').hidden=bench;$('gazebo').parentElement.hidden=bench;
  if(bench){
    for(const id of ['lowering-settings','lowered-shift-settings','lowered-unload-settings','push-settings'])$(id).hidden=true;
    document.querySelectorAll('#push-settings input, #push-settings select, #push-settings button').forEach(c=>{c.disabled=true;});
    document.querySelector('h1').innerHTML='Contact. Air.<br>Contact again.';document.querySelector('.eyebrow').textContent='SENSOR DIAGNOSTIC';
    document.querySelector('.intro-copy').textContent='A small sensor fixture tests Gazebo, the ROS bridge and the existing sampler. No humanoid motor commands.';
    document.querySelector('.envelope .mini-label').textContent='BEFORE ZERO-LOAD CONTROL';document.querySelector('.envelope strong').textContent='3 cycles';document.querySelector('.envelope p').textContent='Fresh empty messages must differ from stale data.';
    document.querySelector('.setup h2').textContent='Check contact sensing';document.querySelector('.setup .section-heading p').textContent='Headless sensor fixture; no humanoid motion.';
    $('mode-note').textContent='Relocate the fixture above the ground, observe fresh no-contact messages, then observe contact returning. All evidence and a state plot are saved.';
    $('run').firstElementChild.textContent='Run sensor diagnostic';document.querySelector('.run-note').textContent='A failed diagnostic blocks claiming that sensor silence means zero support.';
    return;
  }
  const liveLift=$('mode').value==='foot_lift',zero=['zero_load','foot_lift'].includes($('mode').value),near=$('mode').value==='lowered_unload'||zero,combined=['lowered_shift','lowered_unload','zero_load','foot_lift'].includes($('mode').value),lowering=$('mode').value==='lowering'||combined;
  $('lowered-unload-settings').hidden=!near||zero;$('lowered-shift-distance').hidden=near;document.querySelector('label[for="lowered-shift-distance"]').hidden=near;
  $('lowered-shift-settings').hidden=!combined;$('lowering-depth').disabled=combined;if(combined)$('lowering-depth').value='40';
  document.querySelector('#lowered-shift-settings .field-note').textContent=zero?'Lower, transfer toward zero load, return and rise. Supporting-foot contact is mandatory; no lift is commanded.':'Lower 40 mm, settle, shift, hold, center, settle, rise. Both feet stay loaded; no push.';
  $('lowering-settings').hidden=!lowering;$('push-settings').hidden=lowering;
  document.querySelector('#lowering-settings .field-note').textContent=zero?'Supporting-foot contact must remain loaded. Fresh other-foot absence is allowed only in the planned near-zero window.':combined?'Lower 40 mm before transferring weight. Both feet remain in contact; measured loads validate the selected transfer.':'Both feet stay planted and approximately equally loaded. No external force. Joint PID gains and motor caps are unchanged.';
  document.querySelectorAll('#push-settings input, #push-settings select, #push-settings button').forEach(control=>{control.disabled=lowering;});
  document.querySelector('.setup h2').textContent=combined?'Define the combined motion':lowering?'Define the lowering':'Define the push';
  document.querySelector('.setup .section-heading p').textContent=combined?'Lower, shift, center and rise.':lowering?'Centered posture, both feet planted.':'World-frame force on the robot’s base.';
  document.querySelector('.preview-heading .mini-label').textContent=lowering?'POSTURE PREVIEW':'FORCE PREVIEW';
  $('force-arrow').style.display=lowering?'none':'';$('arrow-text').style.display=lowering?'none':'';
  if(lowering){
    const mm=Number($('lowering-depth').value),shiftMm=zero?132.02:near?({20:79.124,10:105.552,5:118.782}[Number($('unload-percent').value)]):Number($('lowered-shift-distance').value),total=near?233:combined?105+2*(8*shiftMm/5):17+2*mm;
    document.querySelector('h1').innerHTML='Lower slowly.<br>Measure every stage.';
    document.querySelector('.eyebrow').textContent='EXPERIMENT / P54 BILATERAL LOWERING';
    document.querySelector('.intro-copy').textContent='Build toward single support one measured posture at a time. Both feet remain planted; no external force.';
    document.querySelector('.envelope .mini-label').textContent='CURRENT LOWERING ENVELOPE';
    document.querySelector('.envelope strong').innerHTML='40 mm <span>max</span>';
    document.querySelector('.envelope p').innerHTML='10 / 20 / 30 / 40 mm milestones<br>Equal support · 0.25× simulation speed';
    $('mode-note').textContent=`Full standing readiness, then ${total} simulated seconds: baseline, slow lowering, hold, rising and recovery. About ${Math.ceil((total+16)/15)}–${Math.ceil((total+16)/15)+1} wall minutes including startup.`;
    $('settling-option').hidden=true;$('run').firstElementChild.textContent=combined?'Run lowered transfer':'Run P54 lowering';
    document.querySelector('.run-note').textContent='One measured run. Planned versus measured height, foot loads and joint speeds are plotted automatically. Gazebo closes after recording; saved plots and history remain.';
    $('diagram-title').textContent='CENTERED CROUCH · SCHEMATIC';$('diagram-axis').textContent='+X';
    $('vector-label').textContent=combined?`40 mm DOWN · ${shiftMm} mm `+$('lowered-shift-side').value.toUpperCase():`${mm} mm DOWN · NO PUSH`;
    $('pulse-preview').textContent=combined?'Lower → shift → center → rise':`${mm} mm lowering + return`;$('impulse-preview').textContent='0 N·s · no force';
    document.querySelector('.preview-footer>div:first-child>span').textContent='Requested motion';
    document.querySelector('.robot-view').setAttribute('aria-label','Illustrative biped crouch; measured motion is shown in the result plot');
    if(combined){
      document.querySelector('.eyebrow').textContent='EXPERIMENT / P54 LOWERED TRANSFER';
      document.querySelector('.envelope p').innerHTML=`40 mm lowered posture · ±${shiftMm} mm shift<br>Both feet loaded · 0.25× simulation speed`;
      document.querySelector('h1').innerHTML='Lower. Shift.<br>Measure the transfer.';
      document.querySelector('.intro-copy').textContent=`Combine the validated 40 mm crouch with a ${shiftMm} mm sideways pelvis motion. Both feet remain planted; measured loads show the transfer.`;
      $('mode-note').textContent=`Full standing readiness, then ${total} simulated seconds: lower, settle, shift, hold, center, settle, rise and recover. Allow about ${Math.ceil(total/15)}–${Math.ceil(total/15)+1} wall minutes including startup.`;
      document.querySelector('.run-note').textContent='One measured combined motion. Plots show height, lateral pelvis motion, measured load shares and joint speeds. No external push or foot lift.';
    }
    if(near){document.querySelector('h1').innerHTML='Shift weight.<br>Measure unloading.';document.querySelector('.intro-copy').textContent=`Target ${$('unload-percent').value}% remaining load on the lighter foot, ${$('lowered-shift-side').value} supporting side. Approximate pelvis travel ${shiftMm.toFixed(1)} mm; actual IK and measured results are saved.`;$('vector-label').textContent=`${$('unload-percent').value}% REMAINING · `+$('lowered-shift-side').value.toUpperCase()+' SUPPORT';}
    if(zero){document.querySelector('.envelope p').textContent='40 mm lowered posture · zero other-foot load · 0.25× simulation speed';document.querySelector('.intro-copy').textContent=`Transfer toward zero other-foot load with ${$('lowered-shift-side').value} support. Uses fresh physics contact states. Fixed-sole reference; no foot lift commanded.`;$('vector-label').textContent='ZERO LOAD · '+$('lowered-shift-side').value.toUpperCase()+' SUPPORT';}
    if(liveLift){document.querySelector('.setup h2').textContent='Define the guarded lift';document.querySelector('.setup .section-heading p').textContent='Unload, lift, land, reload and return.';$('diagram-title').textContent='VERTICAL FOOT LIFT · SCHEMATIC';$('vector-label').textContent=liftMm+' MM LIFT · '+$('lowered-shift-side').value.toUpperCase()+' SUPPORT';$('pulse-preview').textContent='Transfer → lift → land → return';document.querySelector('.envelope .mini-label').textContent='REQUESTED FOOT CLEARANCE';document.querySelector('.envelope strong').textContent=liftMm+' mm';document.querySelector('h1').innerHTML='Transfer. Lift.<br>Land and return.';document.querySelector('.intro-copy').textContent='Measured '+liftMm+' mm vertical foot lift with '+$('lowered-shift-side').value+' support. Fresh sole geometry and contact-state guards stay active.';document.querySelector('.envelope p').textContent=liftMm+' mm lift · 40 mm lowered posture · 0.25× simulation';$('mode-note').textContent=`${liftTotal} simulated seconds after readiness: verified transfer, ${liftMove} s lift, 2 s hold, ${liftMove} s lowering, 2 s recontact, then center and rise. Allow about 20–21 wall minutes.`;$('run').firstElementChild.textContent='Run guarded '+liftMm+' mm lift';document.querySelector('.run-note').textContent='One measured trial. Supporting foot must stay loaded; raised sole must have fresh no-contact evidence. Clearance, landing and full return are audited.';document.querySelector('#lowered-shift-settings .field-note').textContent=`Select the supporting leg. The opposite foot lifts ${liftMm} mm vertically; no forward step or external push.`;}
    return;
  }
  document.querySelector('h1').innerHTML='A small push.<br>A measurable response.';
  document.querySelector('.preview-footer>div:first-child>span').textContent='Requested pulse';
  const visual=$('mode').value==='visual',lateral=$('mode').value==='load_transfer';
  const choices=lateral?['left','right']:['forward','backward'];
  if(!choices.includes(direction))direction=choices[0];
  document.querySelectorAll('.direction').forEach((button,i)=>{button.dataset.direction=choices[i];button.innerHTML=lateral?(i===0?'→ Left <small>+Y</small>':'← Right <small>−Y</small>'):(i===0?'→ Forward <small>+X</small>':'← Backward <small>−X</small>');});
  $('loaded-side-field').hidden=!lateral;
  for(const id of ['force-number','force-range']){$(id).min=lateral?'0':'0.1';$(id).max=lateral?'1':'100';if(Number($(id).value)>Number($(id).max))$(id).value=$(id).max;}
  for(const id of ['duration-number','duration-range']){$(id).max=lateral?'0.2':'0.5';if(Number($(id).value)>Number($(id).max))$(id).value=$(id).max;}
  $('force-low').textContent=lateral?'0 N · reference':'0.1 N';$('force-high').textContent=lateral?'1 N':'100 N';$('duration-high').textContent=lateral?'0.20 s':'0.50 s';
  for(const option of $('balance').options){option.hidden=lateral&&['enhanced','damped'].includes(option.value);option.disabled=option.hidden;}
  if(lateral&&['enhanced','damped'].includes($('balance').value))$('balance').value='on';
  $('balance').options[0].textContent=lateral?'Joint PID + measured load-share feedback':'Joint PID + IMU ankle feedback';
  document.querySelector('.eyebrow').textContent=lateral?'EXPERIMENT / P54 LATERAL TRANSFER':'EXPERIMENT / LEGACY SAGITTAL PUSH';
  document.querySelector('.intro-copy').textContent=lateral?'Ten-joint, 22.28 kg P54 model. Transfer to 70/30, apply a small sideways pulse, and return to center.':'Explore stronger pushes. Recovery may fail; posture guards stop trials that lose standing.';
  document.querySelector('.envelope .mini-label').textContent=lateral?'BILATERAL LOAD TEST':'RECOVERY-LIMIT EXPERIMENTS';
  document.querySelector('.envelope strong').innerHTML=lateral?'1 N <span>max</span>':'100 N <span>max</span>';
  document.querySelector('.envelope p').innerHTML=lateral?'Left & right · world Y<br>37 simulated seconds + settling · 0.25× speed':'Forward & backward · pitch-only legs<br>10 s recovery observation · 0.25× speed';
  $('mode-note').textContent=lateral?'One measured P54 transfer. The pulse is scheduled 13 simulated seconds after standing readiness, during the hold. Choose 0 N for a reference. Gazebo closes after the measured run.':'Visual mode opens Gazebo and stays open until Stop; measured legacy mode runs reference + push.';
  $('diagram-title').textContent=lateral?'BASE LINK · FRONT PROJECTION':'BASE LINK · SIDE VIEW';$('diagram-axis').textContent=lateral?'+Y':'+X';document.querySelector('.robot-view').setAttribute('aria-label',lateral?'Front-projection schematic of the biped and lateral force':'Side-view sketch of the biped with the selected force arrow');
  $('direction-note').textContent=lateral?'Force direction is independent of the loaded-side setting. Both feet must remain grounded.':'Sideways and vertical tests are not part of this legacy profile.';
  $('balance-note').textContent=lateral?'Slow load-share correction: ±4 mm pelvis offset. The same force, torque, posture and sensor guards stay active.':'Original: 0.5 Nm ankle cap; stronger/damped: 1.5 Nm. These are legacy six-joint profiles.';
  $('settling-option').hidden=!visual;
  $('run').firstElementChild.textContent=lateral?'Run P54 transfer':visual?'Open visual simulation':'Run experiment';
  document.querySelector('.run-note').textContent=lateral?'One measured transfer, about 4 minutes including settling. Orange arrow: commanded lateral pulse. Cyan: COM; yellow: ground projection. Results and logs are saved.':visual?'One simulation, one pulse as soon as the visual standing check passes. Gazebo stays open until Stop. Cyan: COM. Yellow: ground projection/trail. After a fall, control stops and Gazebo stays open.':'A fresh no-push reference runs first, then a fresh push trial. Each run saves its own evidence.';
  const f=Number($('force-number').value),d=Number($('duration-number').value),sign=['forward','left'].includes(direction)?1:-1;
  $('vector-label').textContent=`${sign>0?'+':'−'}${Number.isFinite(f)?f.toFixed(2):'—'} N · WORLD ${lateral?'Y':'X'}`;
  $('pulse-preview').textContent=`${f.toFixed(2)} N × ${d.toFixed(2)} s`;
  $('impulse-preview').textContent=`${(f*d).toFixed(3)} N·s`;
  const arrow=$('force-arrow'),text=$('arrow-text');
  arrow.setAttribute('x1',sign>0?'350':'270');arrow.setAttribute('x2',sign>0?'460':'160');
  text.setAttribute('x',sign>0?'405':'215');text.textContent=`${f.toFixed(1)} N`;
  document.querySelectorAll('.direction').forEach(button=>{
    const selected=button.dataset.direction===direction;button.classList.toggle('selected',selected);button.setAttribute('aria-pressed',String(selected));
  });
}
for(const id of ['walk-steps','walk-step-mm','walk-time-scale','walk-transfer-scale','walk-velocity-ff'])$(id).addEventListener('change',preview);
$('lift-mm').addEventListener('change',preview);
$('mode').addEventListener('change',()=>{if($('mode').value!=='load_transfer'&&Number($('force-number').value)<.1){$('force-number').value='.1';$('force-range').value='.1';}preview();});
$('lowering-depth').addEventListener('change',preview);
$('lowered-shift-side').addEventListener('change',preview);
$('lowered-shift-distance').addEventListener('change',preview);
$('unload-percent').addEventListener('change',preview);
$('balance').addEventListener('change',()=>{$('full-settle').checked=['enhanced','damped'].includes($('balance').value);});
for(const name of ['force','duration']){
  $(`${name}-range`).addEventListener('input',event=>{$(`${name}-number`).value=event.target.value;preview();});
  $(`${name}-number`).addEventListener('input',event=>{$(`${name}-range`).value=event.target.value;preview();});
}
document.querySelectorAll('.direction').forEach(button=>button.addEventListener('click',()=>{direction=button.dataset.direction;preview();}));
// All mutations go through the authenticated local server.
async function api(path,payload){
  const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Force-Panel-Token':token},body:JSON.stringify(payload)});
  const body=await response.json();if(!response.ok)throw new Error(body.error||'Request failed');return body;
}
$('experiment-form').addEventListener('submit',async event=>{
  event.preventDefault();if(submitting||state.active_id||closing)return;
  if(!$('experiment-form').reportValidity())return;
  submitting=true;$('run').disabled=true;notice('');
  try{
    const lower=['lowering','lowered_shift','lowered_unload','contact_bench','zero_load','foot_lift_preview','foot_lift','first_step','first_step_preview','alternating_walk','alternating_walk_preview'].includes($('mode').value);
    const result=await api('/api/run',{walk_steps:Number($('walk-steps').value),walk_step_mm:Number($('walk-step-mm').value),walk_time_scale:Number($('walk-time-scale').value),walk_transfer_time_scale:Number($('walk-transfer-scale').value),walk_velocity_feedforward:$('walk-velocity-ff').checked,mode:$('mode').value,lift_mm:Number($('lift-mm').value),direction:lower?'forward':direction,force_n:lower?0:Number($('force-number').value),duration_s:Number($('duration-number').value),balance_enabled:lower?false:$('balance').value!=='off',balance_profile:lower||$('mode').value==='load_transfer'?'original':$('balance').value==='damped'?'damped_response':$('balance').value==='enhanced'?'ankle_response':'original',full_settling:lower?false:$('full-settle').checked,gazebo_gui:['contact_bench','foot_lift_preview','first_step_preview','alternating_walk_preview'].includes($('mode').value)?false:$('gazebo').checked,loaded_side:$('loaded-side').value,lowering_mm:Number($('lowering-depth').value),shift_side:$('lowered-shift-side').value,shift_mm:Number($('lowered-shift-distance').value),unload_percent:Number($('unload-percent').value)});
    selectedId=result.id;await poll();
  }catch(error){notice(error.message);}finally{submitting=false;render();}
});
$('reaudit').addEventListener('click',async()=>{const job=state.jobs.find(j=>j.id===selectedId)||state.jobs[0];try{const r=await api('/api/reaudit',{id:job.id});selectedId=r.id;await poll();}catch(error){notice(error.message);}});
$('stop').addEventListener('click',async()=>{
  if(!state.active_id)return;$('stop').disabled=true;
  try{await api('/api/stop',{id:state.active_id});await poll();}catch(error){notice(error.message);await poll();}
});
$('close-panel').addEventListener('click',async()=>{
  try{await api('/api/shutdown',{});closing=true;$('status-title').textContent='Closing the panel.';$('status-copy').textContent='Any active experiment is being stopped and its simulation closed.';$('settings').disabled=true;$('run').disabled=true;$('stop').disabled=true;$('close-panel').disabled=true;}
  catch(error){notice(error.message);}
});
// Prefer saved job settings when showing results; do not relabel old evidence.
function render(){
  const active=!!state.active_id;
  const activeJob=state.jobs.find(j=>j.id===state.active_id);
  if(activeJob&&lockedJobId!==activeJob.id){
    const r=activeJob.request;$('walk-steps').value=String(r.walk_steps||3);$('walk-step-mm').value=String(r.walk_step_mm||50);$('walk-time-scale').value=String(r.walk_time_scale||1);$('walk-transfer-scale').value=String(r.walk_transfer_time_scale||0);$('walk-velocity-ff').checked=!!r.walk_velocity_feedforward;$('lift-mm').value=String(r.lift_mm||2);direction=r.direction;$('mode').value=r.mode||'experiment';
    $('force-number').value=r.force_n;$('force-range').value=r.force_n;
    $('duration-number').value=r.duration_s;$('duration-range').value=r.duration_s;
    $('balance').value=r.balance_enabled?(r.balance_profile==='damped_response'?'damped':r.balance_profile==='ankle_response'?'enhanced':'on'):'off';$('gazebo').checked=r.gazebo_gui;
    $('full-settle').checked=!!r.full_settling;$('loaded-side').value=r.loaded_side||'left';
    $('unload-percent').value=String(r.unload_percent||20);$('lowering-depth').value=String(r.lowering_mm||10);$('lowered-shift-side').value=r.shift_side||'left';$('lowered-shift-distance').value=String(r.shift_mm||5);
    lockedJobId=activeJob.id;preview();
  }
  if(!active)lockedJobId=null;
  $('settings').disabled=active||closing;$('run').disabled=active||submitting||closing;
  const job=state.jobs.find(j=>j.id===(state.active_id||selectedId))||state.jobs[0];
  $('stop').disabled=!active||job?.status==='stopping'||closing;
  $('reaudit').hidden=active||!job||job.request.mode!=='alternating_walk'||job.status!=='failed'||!!job.request.source_job;
  if(job){
    $('timeline').hidden=['visual','foot_lift_preview','first_step_preview','alternating_walk_preview'].includes(job.request.mode);
    const complete=job.status==='completed',passed=job.result?.passed;
    const savedLabel=['alternating_walk','alternating_walk_preview'].includes(job.request.mode)?`${job.request.walk_steps||3} alternating steps`+(job.request.mode.endsWith('preview')?' OFFLINE preflight':''):['first_step','first_step_preview'].includes(job.request.mode)?`50 mm forward step${job.request.mode==='first_step_preview'?' OFFLINE preflight':''}, ${job.request.shift_side} support`:job.request.mode==='foot_lift'?`measured ${job.request.lift_mm||2} mm lift, ${job.request.shift_side} support`:job.request.mode==='foot_lift_preview'?`OFFLINE ${job.request.lift_mm||2} mm lift preview, ${job.request.shift_side} support`:job.request.mode==='zero_load'?`zero-load transition, ${job.request.shift_side} support`:job.request.mode==='contact_bench'?'sensor contact/air/contact diagnostic':job.request.mode==='lowered_unload'?`${job.request.unload_percent}% remaining load, ${job.request.shift_side} support`:job.request.mode==='lowered_shift'?`${job.request.shift_mm||5} mm ${job.request.shift_side||'left'} transfer at 40 mm lowering`:job.request.mode==='lowering'?`${job.request.lowering_mm||10} mm centered lowering`:job.request.mode==='load_transfer'?'P54 lateral pulse / transfer':'legacy push experiment';
    $('status-title').textContent=complete?(['foot_lift_preview','first_step_preview','alternating_walk_preview'].includes(job.request.mode)?'Offline preview complete.':'Experiment complete.'):job.status==='failed'?'Experiment stopped with an error.':job.status==='stopped'?'Experiment stopped.':job.status==='interrupted'?'Previous run interrupted.':job.message;
    $('status-copy').textContent=complete?`Saved run: ${savedLabel}. `+(passed?(['foot_lift_preview','first_step_preview','alternating_walk_preview'].includes(job.request.mode)?'The numerical checks passed. Inspect the saved predictions below; no lift was measured.':'The recorded checks passed. Inspect the saved measurements below.'):job.message+'; inspect the saved results below.'):job.message;
    $('status-badge').textContent=complete?(passed?(['foot_lift_preview','first_step_preview','alternating_walk_preview'].includes(job.request.mode)?'Offline checked':'Passed'):'Checks not met'):job.status==='running'?'Running':job.status==='stopping'?'Stopping':job.status==='stopped'?'Stopped':job.status==='interrupted'?'Interrupted':'Error';
    $('status-badge').className='status-badge '+(complete&&!passed?'warn':job.status);
    const elapsed=Math.max(0,Math.floor(((job.finished_at?new Date(job.finished_at):new Date())-new Date(job.created_at))/1000));
    $('elapsed').textContent=`${Math.floor(elapsed/60)}m ${String(elapsed%60).padStart(2,'0')}s elapsed · wall clock`;
    $('run-id').textContent=job.id;$('log').textContent=job.log||'Waiting for runner output…';
    document.querySelectorAll('#timeline li').forEach((item,i)=>{item.classList.toggle('done',complete||job.status==='running'&&i<job.step);item.classList.toggle('current',job.status==='running'&&i===job.step);});
    const isWalk=job.request.mode==='alternating_walk',isWalkPreview=job.request.mode==='alternating_walk_preview';
    const isStep=job.request.mode==='first_step',isStepPreview=job.request.mode==='first_step_preview';
    const isPreview=job.request.mode==='foot_lift_preview',isBench=job.request.mode==='contact_bench';
    const isCombined=['lowered_shift','lowered_unload','zero_load','foot_lift'].includes(job.request.mode),isLower=job.request.mode==='lowering'||isCombined,isLoad=job.request.mode==='load_transfer'||isLower;
    document.querySelectorAll('#timeline li').forEach((item,i)=>{item.textContent=(isWalk?['Stand / transfer','Lift','Alternate','Land / reload','Results']:isStep?['Stand / unload','Lift','Advance','Land / reload','Results']:isBench?['Contact baseline','Air phase','Re-contact','Repeat cycles','Results']:job.request.mode==='foot_lift'?['Stand / transfer','Confirm unloaded','Lift / hold','Land / return','Results']:isCombined?['Stand / lower','Shift sideways','Hold','Center / rise','Results']:isLower?['Standing','Lower pelvis','Hold','Rise / return','Results']:isLoad?['Standing','Shift weight','Hold / pulse','Return','Results']:['Reference','Push warmup','Baseline','Push & recover','Results'])[i];});
    $('peak-label').textContent=isLoad?'Peak body tilt':'Peak pitch change';$('recovery-label').textContent=isLoad?'Pelvis return':'Return-band check';
    const result=isWalk||isWalkPreview||isStep||isStepPreview||isPreview||isBench||isLoad?job.result:job.result?.push;$('results').hidden=!result||!complete;
    $('lift-clearance-result').hidden=!['foot_lift','first_step'].includes(job.request.mode)||!complete||!(result?.foot_lift||result?.first_step);
    if(['foot_lift','first_step'].includes(job.request.mode)&&(result?.foot_lift||result?.first_step)){const lift=result.foot_lift||result.first_step;$('lift-clearance').textContent=lift.held_minimum_clearance_m==null?'No held evidence':(lift.held_minimum_clearance_m*1000).toFixed(3)+' mm';}
    $('zero-force-result').hidden=job.request.mode!=='zero_load'||!complete||!result;
    if(job.request.mode==='zero_load'&&complete&&result?.zero_load_transition)$('zero-force').textContent=Number(result.zero_load_transition.measured_held_unloaded_force_n).toFixed(4)+' N';
    $('transfer-metrics').hidden=!isCombined||!complete||!result;
    $('contact-result-label').textContent=isPreview?'Execution':'Foot-contact evidence';
    if(result&&complete){
      if(isWalk||isWalkPreview){$('peak-label').textContent=isWalk?'Verified alternating placements':'Planned alternating placements';$('peak').textContent=isWalk?result.alternating_walk.verified_steps+' / '+result.alternating_walk.step_count:String(result.step_count||3);$('recovery-label').textContent=isWalk?'Measured COM forward progress':'Execution';$('recovery').textContent=isWalk?(result.alternating_walk.measured_com_forward_progress_m*1000).toFixed(2)+' mm':'Offline only';$('contacts').textContent=isWalk?(result.checks.contacts_geometry?'Contacts / geometry valid':'Checks not met'):'No simulation';}
      else if(isStep||isStepPreview){$('peak-label').textContent=isStep?'Measured foot advance':'Planned foot advance';$('peak').textContent=isStep?(result.first_step.measured_final_advance_m*1000).toFixed(3)+' mm':'50 mm';$('recovery-label').textContent=isStep?'Landed foot final load':'Execution';$('recovery').textContent=isStep?(result.first_step.final_landed_foot_load_fraction*100).toFixed(2)+'%':'Offline only';$('contacts').textContent=isStep?(result.first_step.landing_contact_confirmed?'Landing confirmed':'Landing not confirmed'):'No simulation';}
      else if(isPreview){$('peak-label').textContent='Planned clearance';$('peak').textContent=(result.predicted_clearance_m*1000).toFixed(2)+' mm';$('recovery-label').textContent='Predicted support COM margin';$('recovery').textContent=(result.minimum_predicted_support_margin_m*1000).toFixed(2)+' mm';$('contacts').textContent='Offline only; no measured lift';}
      else if(isBench){$('peak-label').textContent='Fresh air readings';$('peak').textContent=String(result.counts.no_contact);$('recovery-label').textContent='Raw sensor air readings';$('recovery').textContent=String(result.raw_counts.no_contact);$('contacts').textContent=result.passed?'Contact returned':'Diagnostic not passed';}
      else{
      $('peak').textContent=`${((isLoad?result.sensors.max_pose_tilt_rad:result.peak_pitch_change_rad)*180/Math.PI).toFixed(3)}°`;
      $('recovery').textContent=(isLoad?result.sensors.pelvis_target_and_return_passed:result.final_second_within_return_band)?'Within bands':'Not met';
      $('contacts').textContent=(isLoad?result.shift.contacts_fresh_and_grounded:result.continuous_contact_evidence)?'Both feet fresh':'Check failed';
      if(['zero_load','foot_lift'].includes(job.request.mode))$('contacts').textContent=result.shift.contact_policy_passed?'Physics states valid':'Contact-state check failed';
      if(isLower){$('peak-label').textContent='Measured lowering';$('peak').textContent=`${(result.lowering.measured_lowering_m*1000).toFixed(3)} mm`;$('recovery-label').textContent='Final height offset';$('recovery').textContent=`${(result.lowering.max_final_height_offset_m*1000).toFixed(3)} mm`;}
      if(isCombined){$('lateral-result').textContent=`${(result.sensors.measured_pelvis_shift_m*1000).toFixed(3)} mm`;$('com-result').textContent=`${(result.shift.measured_com_shift_relative_feet_m*1000).toFixed(3)} mm`;$('load-result').textContent=`${(result.lowering.measured_left_share_change*100).toFixed(3)} pp`;}
      }
      $('evidence-path').textContent=job.output_relative;
      $('summary-link').href=`/api/jobs/${job.id}/summary`;
      $('plot-link').href=`/api/jobs/${job.id}/plot`;
      const image=$('result-plot');image.alt=isPreview?'Offline lift trajectory, predicted torques and support margin':'Recorded experiment response plot';image.hidden=!job.plot_ready;
      if(job.plot_ready&&image.dataset.job!==job.id){image.src=`/api/jobs/${job.id}/plot`;image.dataset.job=job.id;}
      if(job.plot_error)notice(job.plot_error);
    }
  }
  renderRoadmap(active);
  const historyKey=state.jobs.map(j=>`${j.id}:${j.status}:${j.result?.passed}`).join('|')+':'+active;
  if(historyVersion===historyKey)return;historyVersion=historyKey;
  const history=$('history');history.replaceChildren();
  if(!state.jobs.length){history.className='history-empty';history.textContent='Your experiments will appear here.';return;}
  history.className='';
  for(const item of state.jobs){
    const button=document.createElement('button');button.className='history-row';button.disabled=active;
    const request=item.request;const parts=[new Date(item.created_at).toLocaleTimeString([], {hour:'2-digit',minute:'2-digit'}),`${['forward','left'].includes(request.direction)?'+':'−'}${Number(request.force_n).toFixed(1)} N · ${Number(request.duration_s).toFixed(2)} s`,request.mode==='load_transfer'?(request.balance_enabled?'P54 · load feedback':'P54 · joint PID'):request.balance_enabled?(request.balance_profile==='damped_response'?'PID + IMU · damped':request.balance_profile==='ankle_response'?'PID + IMU · experimental':'PID + IMU'):'Joint PID only',item.status==='completed'?(item.result?.passed?'Passed':'Checks not met'):item.status];
    if(request.mode==='lowered_shift'){parts[1]=`40 mm down · ${request.shift_mm||5} mm `+request.shift_side;parts[2]='P54 · lowered transfer';}
    if(request.mode==='lowered_unload'){parts[1]=`${request.unload_percent}% remaining · ${request.shift_side} support`;parts[2]='P54 · lowered unloading';}
    if(request.mode==='zero_load'){parts[1]=`Zero load · ${request.shift_side} support`;parts[2]='P54 · fixed-sole transition';}
    if(['alternating_walk','alternating_walk_preview'].includes(request.mode)){parts[1]=`${request.walk_steps||3} steps · ${request.walk_step_mm||50} mm · ${1/(request.walk_time_scale||1)}× motion · transfer ${(request.walk_transfer_time_scale||request.walk_time_scale||1)}`;parts[2]=request.source_job?'Re-audit of saved measurements':request.mode.endsWith('preview')?'OFFLINE walking preflight':'Measured slow gait';}
    if(['first_step','first_step_preview'].includes(request.mode)){parts[1]='50 mm forward step · '+request.shift_side+' support';parts[2]=request.mode==='first_step'?'Measured placement / reload':'OFFLINE preflight';}
    if(request.mode==='foot_lift'){parts[1]=`${request.lift_mm||2} mm measured lift · ${request.shift_side} support`;parts[2]='P54 · guarded lift / landing';}
    if(request.mode==='foot_lift_preview'){parts[1]=`OFFLINE ${request.lift_mm||2} mm lift · ${request.shift_side} support`;parts[2]='Predicted only';if(item.status==='completed'&&item.result?.passed)parts[3]='Offline checked';}
    if(request.mode==='contact_bench'){parts[1]='Contact / air / contact';parts[2]='Sensor fixture';}
    if(request.mode==='lowering'){parts[1]=`${request.lowering_mm} mm lowering + return`;parts[2]='P54 · equal bilateral support';}
    for(const text of parts){const span=document.createElement('span');span.textContent=text;button.append(span);}
    button.addEventListener('click',()=>{selectedId=item.id;render();$('status-title').scrollIntoView({behavior:'smooth',block:'center'});});history.append(button);
  }
}
function renderRoadmap(active){
  const version=JSON.stringify(state.roadmap)+':'+active+':'+closing;if(version===roadmapVersion)return;roadmapVersion=version;
  const container=$('roadmap-stages');container.replaceChildren();
  for(const [i,stage] of (state.roadmap||[]).entries()){
    const row=document.createElement('div');row.className='roadmap-row';
    const label=document.createElement('span');label.textContent=`${i+1}. ${stage.title}`;
    const badge=document.createElement('span');badge.className='roadmap-status '+(stage.status==='verified'?'verified':'');badge.textContent=stage.status;
    row.append(label,badge);
    if(stage.mode){const button=document.createElement('button');button.type='button';button.textContent='Choose';button.disabled=active||closing;button.setAttribute('aria-label','Choose '+stage.title);button.addEventListener('click',()=>{$('mode').value=stage.mode;if(stage.walk_steps)$('walk-steps').value=String(stage.walk_steps);if(stage.walk_step_mm)$('walk-step-mm').value=String(stage.walk_step_mm);if(stage.walk_time_scale)$('walk-time-scale').value=String(stage.walk_time_scale);$('walk-transfer-scale').value=String(stage.walk_transfer_time_scale||0);$('walk-velocity-ff').checked=!!stage.walk_velocity_feedforward;if(stage.lift_mm)$('lift-mm').value=String(stage.lift_mm);if(stage.unload_percent)$('unload-percent').value=String(stage.unload_percent);if(stage.lowering_mm)$('lowering-depth').value=String(stage.lowering_mm);if(stage.shift_side)$('lowered-shift-side').value=stage.shift_side;if(stage.shift_mm)$('lowered-shift-distance').value=String(stage.shift_mm);preview();$('mode').scrollIntoView({behavior:'smooth',block:'center'});});row.append(button);}
    container.append(row);
  }
}
async function poll(){
  if(closing)return;
  try{const response=await fetch('/api/state');if(!response.ok)throw new Error('Panel connection failed');state=await response.json();if(connectionFailed){notice('');connectionFailed=false;}render();}
  catch(error){connectionFailed=true;notice('Cannot reach the local panel. Start scripts/start_force_panel.sh if it has stopped.');}
}
preview();poll();setInterval(poll,1000);
