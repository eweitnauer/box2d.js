(function(){

var Test = function() {
	this.__constructor(arguments);
}

Test.__constructor = function(canvas) {
	this._canvas = canvas;
	this._paused = true;
	this._fps = 200;
	this._dbgDraw = new b2DebugDraw();
	
	this._velocityIterationsPerSecond = 300;
	this._positionIterationsPerSecond = 200;
	
	// sublcasses expect visual area inside 64x36
	this._dbgDraw.m_drawScale = Math.min(canvas.width/64, canvas.height/36);
	this._dbgDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit | b2DebugDraw.e_centerOfMassBit);
	this._world = this.createWorld();
}

Test.prototype.log = function(arg) {
    if(typeof(window.console) != 'undefined') {
        console.log(arg);
    }
};

Test.prototype.destroy = function() {
	this.pause();
	this._canvas = null;
	this._dbgDraw = null;
	this._world = null;
}

Test.prototype.createWorld = function(){
	var m_world = new b2World(new b2Vec2(0.0, -9.81), true);
	var m_physScale = 1;
	m_world.SetWarmStarting(true);
	
	// Create border of boxes
	var wall = new b2PolygonShape();
	var wallBd = new b2BodyDef();
	
	// Left
	wallBd.position.Set( -9.5 / m_physScale, 36 / m_physScale / 2);
	wall.SetAsBox(10/m_physScale, 40/m_physScale/2);
	this._wallLeft = m_world.CreateBody(wallBd);
	this._wallLeft.CreateFixture2(wall);
	// Right
	wallBd.position.Set((64 + 9.5) / m_physScale, 36 / m_physScale / 2);
	this._wallRight = m_world.CreateBody(wallBd);
	this._wallRight.CreateFixture2(wall);
	// Top
	wallBd.position.Set(64 / m_physScale / 2, (36 + 9.5) / m_physScale);
	wall.SetAsBox(68/m_physScale/2, 10/m_physScale);
	this._wallTop = m_world.CreateBody(wallBd);
	this._wallTop.CreateFixture2(wall);	
	// Bottom
	wallBd.position.Set(64 / m_physScale / 2, -9.5 / m_physScale);
	this._wallBottom = m_world.CreateBody(wallBd);
	this._wallBottom.CreateFixture2(wall);
	
	return m_world;
};

Test.prototype.draw = function() {
	var c = this._canvas.getContext("2d");
	
	this._dbgDraw.SetSprite(c);
	if(this._world) {
		this._world.SetDebugDraw(this._dbgDraw);
		this._world.DrawDebugData();
	}
	
	c.fillStyle = "black";
	if(this._paused) {
		c.fillText("paused", 5, 15);
	} else {
		c.fillText("FPS: " + this._fpsAchieved, 5, 15);
	}
}

Test.prototype.step = function(delta) {
	if(!this._world)
		return;
		
	this._world.ClearForces();
	
	var delta = (typeof delta == "undefined") ? 1/this._fps : delta;
	
	this._world.Step(delta, delta * this._velocityIterationsPerSecond, delta * this._positionIterationsPerSecond);	
}

Test.prototype._update = function() {
	// derive passed time since last update. max. 10 secs
	var time = new Date().getTime();
	delta = (time - this._lastUpdate) / 1000;
	this._lastUpdate = time;
	if(delta > 10)
		delta = 1/this._fps;
		
	// see this._updateFPS
	this._fpsCounter++;
	
	this.step(delta);
	this.draw();
	if(!this._paused) {
		that = this;
		this._updateTimeout = window.setTimeout(function(){that._update()});
	}
}

Test.prototype._updateFPS = function() {
	this._fpsAchieved = this._fpsCounter;
	this.log("fps: " + this._fpsAchieved);
	this._fpsCounter = 0;
	
	if(!this._paused) {
		that = this;
		this._updateFPSTimeout = window.setTimeout(function(){that._updateFPS()}, 1000);
	}
}

Test.prototype.resume = function() {
	if(this._paused) {
		this._paused = false;
		this._lastUpdate = 0;
		this._update();
		this._updateFPS();
	}
}

Test.prototype.pause = function() {
	this._paused = true;
	
	window.clearTimeout(this._updateTimeout);
	window.clearTimeout(this._updateFPSTimeout);
}

Test.prototype.isPaused = function() {
	return this._paused;
}

window.b2jsTest = Test;
	
})();