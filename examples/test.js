jQuery(function($){

var canvas = document.getElementById('canvas');
var ctx = canvas.getContext('2d');
var c_width = 55.0;
var ppm = canvas.width/c_width;
var c_height = canvas.height/ppm;
ctx.setTransform(ppm, 0, 0, -ppm, 0, canvas.height);  

var gravity = new b2Vec2(0.0, -9.8);
var world = new b2World(gravity, true);
window.world = world;

var groundBodyDef = new b2BodyDef();
groundBodyDef.position.Set(c_width/2.0, 3.0);
var groundShape = b2PolygonShape.AsBox(c_width*1.0, 5.0);
var groundFixture = new b2FixtureDef();
groundFixture.restitution = 0.0;
groundFixture.friction = 0.5;
groundFixture.density = 1.0;
groundFixture.shape = groundShape;

var groundBody = world.CreateBody(groundBodyDef);
groundBody.CreateFixture(groundFixture);
groundBody.w = c_width*1.0;
groundBody.h = 5.0;

var bodies = [groundBody];
var explosionParticles = [];

function spawn(x, y, a) {
    var bodyDef = new b2BodyDef();
    bodyDef.type = b2Body.b2_dynamicBody;
    bodyDef.position.Set(x, y);
    bodyDef.angle = a;
    var body = world.CreateBody(bodyDef);
    body.w = 1.0;
    body.h = 1.0;
    var shape = new b2PolygonShape.AsBox(body.w, body.h);
    var fixtureDef = new b2FixtureDef();
    fixtureDef.restitution = 0.0;
    fixtureDef.density = 2.0;
    fixtureDef.friction = 0.9;
    fixtureDef.shape = shape;
    
    body.CreateFixture(fixtureDef);
    //body.SetMassFromShapes();
    bodies.push(body);
}

function explode(x, y) {
	return;
	var count = 50;
    for(var i = 0; i < count; i++) {
        var a = 2*Math.PI/count*i;
        var vx = Math.cos(a);
        var vy = Math.sin(a);
        var bodyDef = new b2BodyDef();
        bodyDef.position.Set(x+vx, y+vy);
        bodyDef.isBullet = true;
        bodyDef.type = b2Body.b2_dynamicBody;
        var body = world.CreateBody(bodyDef);
        var fd = new b2FixtureDef();
        fd.restitution = 0.0;
        fd.density = 100.0;
        fd.friction = 0.0;
        fd.shape = new b2PolygonShape.AsBox(0.1, 0.1);
        body.CreateFixture(fd);
        body.ApplyImpulse({x: vx*500, y:vy*500}, {x:x, y:y});
        body.w = 0.1;
        body.h = 0.1;
        explosionParticles.push(body);
    }
}
$(canvas).click(function (e){
    var o = $(canvas).offset();
    var x = (e.pageX-o.left)/ppm;
    var y = (canvas.height-e.pageY+o.top)/ppm;
    explode(x, y);
//    spawn((e.pageX-o.left)/ppm, (canvas.height-e.pageY+o.top)/ppm);
});
//debugger;

for(var i = 0; i < 50; i ++) {
    spawn(c_width/2 + Math.sin(i/10) * 8, c_height + i * 2.4, 0);
}

var frame = 0;
var fps = new Engine.FPSCounter(ctx);
window.bbb = bodies[1];
window.setInterval(function() {
    frame ++;
    if(frame%20 == 0) {
//        spawn(c_width/2, c_height);
    }
//    debugger;
    //bodies[0].ApplyForce(new b2Vec2(0.1, 0.1));
//    console.log(window.bbb.m_xf.position.x);
    world.Step(1.0/60.0, 10);
//    console.log(window.bbb.m_xf.position.x);
    world.ClearForces();
    //ctx.clearRect(0.0, 0.0, canvas.width, canvas.height);
    ctx.fillStyle = 'white';
    ctx.fillRect(0, 0, c_width, c_height);
    ctx.fillStyle = 'red';
    for(var i = 0; i < explosionParticles.length; i++){
        var body = explosionParticles[i];
        var t = body.m_xf;
        ctx.translate(t.position.x, t.position.y)
        ctx.beginPath();
        ctx.arc(0, 0, 0.1, 0, Math.PI*2, true);
        ctx.closePath();
        ctx.fill();
        ctx.translate(-t.position.x, -t.position.y)
    }
    ctx.fillStyle = 'black';
    for(var i = 0; i < bodies.length; i++){
        var body = bodies[i];
        var t = body.m_xf;
        ctx.translate(t.position.x, t.position.y)
        ctx.rotate(body.GetAngle());
        ctx.fillRect(-body.w, -body.h, body.w*2, body.h*2);
        ctx.rotate(-body.GetAngle());
        ctx.translate(-t.position.x, -t.position.y)
    }
    ctx.save();
    ctx.setTransform(1,0,0,1,0,0);
    fps.draw();
    ctx.restore();
}, 1000/30);



});
jQuery.noConflict();
