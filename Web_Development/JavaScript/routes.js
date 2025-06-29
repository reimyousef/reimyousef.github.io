/*
This is a helper file for different calls used througout the JavaScript files.
*/

// ───────────────────────────────────────────────────────────────
// routes.js – page guards, APIs, authentication
// ───────────────────────────────────────────────────────────────
const path   = require("path");
const bcrypt = require("bcryptjs");

const util = require("util");

module.exports = function registerRoutes(app, db, PUBLIC_DIR) {

  /* ───────── helpers: page senders / guards ───────── */
  const sendPage = file => (_req, res) =>
    res.sendFile(path.join(PUBLIC_DIR, "pages", file));

  const sendIfLoggedIn = file => (req, res) => {
    if (!req.session?.studentID) return res.redirect("/login.html");
    res.sendFile(path.join(PUBLIC_DIR, "pages", file));
  };

  const sendIfAdmin = file => (req, res) => {
    if (!req.session?.studentID) return res.redirect("/login.html");
    if (!req.session?.isAdmin)   return res.redirect("/landingPage.html");
    res.sendFile(path.join(PUBLIC_DIR, "pages", file));
  };
  const util = require("util");

  db.runAsync = util.promisify(db.run.bind(db));
  db.getAsync = util.promisify(db.get.bind(db));
  db.allAsync = util.promisify(db.all.bind(db));

  /* ───────── schema bootstrap ───────── */
  function createTable(sql) { db.run(sql, err => { if (err) console.error(err.message); }); }
  db.serialize(() => {
    createTable(`CREATE TABLE IF NOT EXISTS Users(
      UserID INTEGER PRIMARY KEY AUTOINCREMENT,
      FirstName TEXT, LastName TEXT, Email TEXT UNIQUE,
      StudentID TEXT UNIQUE, PhoneNumber TEXT,
      PasswordHash TEXT NOT NULL,
      NumFormsFilled INTEGER DEFAULT 0,
      IsAdmin BOOLEAN DEFAULT 0
    )`);

    createTable(`CREATE TABLE IF NOT EXISTS SurveyRequests(
      RequestID INTEGER PRIMARY KEY AUTOINCREMENT,
      Title TEXT, Link TEXT, Name TEXT, StudentNumber TEXT,
      RequestedCount INTEGER, IsVerified5thYear BOOLEAN,
      IsApproved BOOLEAN
    )`);

    createTable(`CREATE TABLE IF NOT EXISTS Sessions(
      SessionID INTEGER PRIMARY KEY AUTOINCREMENT,
      Title TEXT, Description TEXT, Date TEXT,
      StartTime TEXT, EndTime TEXT, Location TEXT,
      Link TEXT, Capacity INTEGER, Category TEXT,
      Visibility TEXT, Attachments TEXT,
      NumSignedUp INTEGER DEFAULT 0,
      SignedUpIDs TEXT NOT NULL DEFAULT '[]'
    )`);
  });

  /* ───────── AUTH ───────── */

  /** sign-up (HTML form) */
  app.post("/signup", (req, res) => {
    const { FirstName, LastName, Email, StudentID, PhoneNumber, Password } = req.body;
    const hash = bcrypt.hashSync(Password, 10);
    db.run(
      `INSERT INTO Users
         (FirstName,LastName,Email,StudentID,PhoneNumber,PasswordHash)
       VALUES (?,?,?,?,?,?)`,
      [FirstName, LastName, Email, StudentID, PhoneNumber, hash],
      err => err
        ? res.send("Sign-up error: " + err.message)
        : res.redirect("/login.html")
    );
  });

  /** AJAX login */
  app.post("/api/login", (req, res) => {
    const { StudentID, Password } = req.body;

    db.get("SELECT * FROM Users WHERE StudentID = ?", [StudentID], (err, row) => {
      if (err)   return res.status(500).json({ error: "Login error" });
      if (!row || !bcrypt.compareSync(Password, row.PasswordHash))
        return res.status(401).json({ error: "Invalid Student ID or password" });

      req.session.studentID = row.StudentID;
      req.session.isAdmin   = !!row.IsAdmin;   // camel-case in the session
      req.session.firstName = row.FirstName;
      req.session.lastName  = row.LastName;
      res.json({ ok: true });
    });
  });

  /** logout */
  app.post("/logout", (req, res) => req.session.destroy(() => res.json({ ok: true })));

  /* -----------------------------------------------------------
     SURVEY-REQUEST APIs
  ----------------------------------------------------------- */
  app.get("/api/survey-requests", (_req, res) => {
    db.all(
      `SELECT
         RequestID        AS id,
         Title            AS title,
         Link             AS link,
         Name             AS name,
         StudentNumber    AS studentNumber,
         RequestedCount   AS requestedCount,
         IsVerified5thYear AS isVerified5thYear
       FROM SurveyRequests
       WHERE IsApproved IS NULL
       ORDER BY RequestID DESC`,
      (err, rows) => err
        ? res.status(500).json({ error: err.message })
        : res.json(rows)
    );
  });

  /* -----------------------------------------------------------
     SINGLE  /api/user/me   (one route only, names aligned)
  ----------------------------------------------------------- */
  app.get("/api/user/me", (req, res) => {
    if (!req.session?.studentID)
      return res.status(401).json({ error: "Login required" });

    db.get(
      `SELECT FirstName, LastName, StudentID, NumFormsFilled, IsAdmin
         FROM Users WHERE StudentID = ?`,
      [req.session.studentID],
      (err, row) => {
        if (err)  return res.status(500).json({ error: err.message });
        if (!row) return res.status(404).json({ error: "User not found" });

        // keep session flag current
        req.session.isAdmin = !!row.IsAdmin;

        /* Return BOTH spellings so existing code keeps working */
        res.json({
          firstName:      row.FirstName,
          lastName:       row.LastName,
          studentID:      row.StudentID,
          StudentID:      row.StudentID,      // legacy, if anything needs it
          numFormsFilled: row.NumFormsFilled,
          isAdmin:        !!row.IsAdmin,      // camel-case
          IsAdmin:        !!row.IsAdmin       // original Pascal-case
        });
      }
    );
  });

  /* bulk approve / reject */
  function bulkUpdate(isApprove) {
    return (req, res) => {
      const ids = req.body.ids || [];
      if (!ids.length) return res.status(400).end();
      const placeholders = ids.map(() => "?").join(",");
      db.run(
        `UPDATE SurveyRequests SET IsApproved = ${isApprove ? 1 : 0}
         WHERE RequestID IN (${placeholders})`,
        ids,
        function (err) {
          if (err) return res.status(500).json({ error: err.message });
          res.json({ updated: this.changes });
        }
      );
    };
  }
  app.post("/approve", bulkUpdate(true));
  app.post("/reject",  bulkUpdate(false));

  /* -----------------------------------------------------------
     SESSION APIs
  ----------------------------------------------------------- */
  const SESS_TABLE = "Sessions";

  // create
  app.post(["/api/session", "/api/sessions"], (req, res) => {
    const {
      Title, Description, Date,
      StartTime, EndTime, Location,
      Link, Capacity, Category,
      Visibility, Attachments
    } = req.body;

    db.run(
      `INSERT INTO ${SESS_TABLE}
         (Title, Description, Date, StartTime, EndTime,
          Location, Link, Capacity, Category,
          Visibility, Attachments)
       VALUES (?,?,?,?,?,?,?,?,?,?,?)`,
      [
        Title, Description, Date,
        StartTime, EndTime,
        Location, Link || null,
        Capacity, Category,
        Visibility || null,
        Attachments || null
      ],
      function (err) {
        if (err) return res.status(500).json({ error: err.message });
        res.json({ sessionID: this.lastID });
      }
    );
  });

  // list
app.get(["/api/session", "/api/sessions"], (req, res) => {
    const vis = req.query.visibility;
    const sql = vis
      ? `SELECT * FROM ${SESS_TABLE} WHERE Visibility = ? ORDER BY Date, StartTime`
      : `SELECT * FROM ${SESS_TABLE} ORDER BY Date, StartTime`;
    db.all(sql, vis ? [vis] : [], (err, rows) =>
      err ? res.status(500).json({ error: err.message }) : res.json(rows)
    );
  });

app.get("/api/inquiries", async (req, res) => {
  try {
    const inquiries = await db.allAsync(`SELECT * FROM inquiries ORDER BY TimeSubmitted ASC`);
    const replies = await db.allAsync(`SELECT * FROM replies`);
    // Group replies under inquiries
  
    const inquiriesWithReplies = inquiries.map(inq => {
      return {
        ...inq,
        replies: replies.filter(r => r.inquiryID === inq.inquiryID)
      };
    });
    res.json(inquiriesWithReplies);
  } catch (err) {
    res.status(500).json({ error: err.message });
  }
});

app.delete("/api/inquiries/:id", async (req, res) => {
  const studentID = req.session.studentID;
  try {
    const inquiryID = req.params.id;
    if (!studentID) return res.status(401).json({ error: "Not logged in" });
    const result = await db.getAsync(`SELECT * FROM inquiries WHERE inquiryID = ?`, [inquiryID]);
    
    if (!result) return res.status(404).json({ error: "Inquiry not found" });

    if (result.author_id !== studentID && !req.session.isAdmin) {
      return res.status(403).json({ error: "Unauthorized" });
    }
    await db.runAsync(
      `DELETE FROM replies WHERE inquiryID = ?`, [inquiryID] );
    await db.runAsync(`DELETE FROM inquiries WHERE inquiryID = ?`, [inquiryID]);
    res.json({ success: true });
  } catch (err) {
    res.status(500).json({ error: err.message });
  }
});

app.delete("/api/replies/:id", async (req, res) => {

  try {
    const replyID = req.params.id;
 

    const studentID = req.session.studentID;
    if (!studentID) return res.status(401).json({ error: "Not logged in" });

    const reply = await db.getAsync(`SELECT * FROM replies WHERE replyID = ?`, [replyID]);
  
    if (!reply) return res.status(404).json({ error: "Reply not found" });

    if (reply.author_id !== studentID && !req.session.isAdmin) {
      return res.status(403).json({ error: "Unauthorized" });
    }

    await db.runAsync(`DELETE FROM replies WHERE replyID = ?`, [replyID]);
    res.json({ success: true });
  } catch (err) {
    res.status(500).json({ error: err.message });
  }
});

app.post('/api/inquiries', async (req, res) => {
  const studentID = req.session.studentID;
  try {
    const { Filter } = await import('bad-words');
    const filter = new Filter();
    const { userName, inquiry } = req.body;
    const cleanInquiry = filter.clean(inquiry);

    const insert = await new Promise((resolve, reject) => {
      db.run(
        `INSERT INTO inquiries (userName, inquiry, TimeSubmitted, author_id)
         VALUES (?, ?, datetime('now'), ?)`,
        [ userName, cleanInquiry, studentID ],
        function (err) {
          if (err) return reject(err);
          resolve({ lastID: this.lastID });
        }
      );
    });

    const inserted = await db.getAsync(
      `SELECT * FROM inquiries WHERE inquiryID = ?`,
      [insert.lastID]
    );

    res.json(inserted);
  } catch (err) {
    console.error("❌ Failed to insert inquiry:", err);
    res.status(500).json({ error: err.message });
  }
});


app.post('/api/replies', async (req, res) => {
  const studentID = req.session.studentID;
  try {
    const { inquiryID, userName, reply, timeReplied } = req.body;
    const { Filter } = await import('bad-words');
    const filter = new Filter();
    const cleanReply = filter.clean(reply);

    const insert = await new Promise((resolve, reject) => {
      db.run(
        `INSERT INTO replies (inquiryID, userName, reply, timeReplied, author_id)
         VALUES (?, ?, ?, ?, ?)`,
        [inquiryID, userName, cleanReply, timeReplied, studentID],
        function (err) {
          if (err) return reject(err);
          resolve({ lastID: this.lastID });
        }
      );
    });
    

    const inserted = await db.getAsync(
      `SELECT * FROM replies WHERE ReplyID = ?`,
      [insert.lastID]
    );


    res.json(inserted);
  } catch (err) {
    console.error("❌ Failed to insert reply:", err);
    res.status(500).json({ error: err.message });
  }
});

  // update
  app.put(["/api/session/:id", "/api/sessions/:id"], (req, res) => {
    const id = +req.params.id;
    const fields = [
      "Title", "Description", "Date", "StartTime", "EndTime",
      "Location", "Link", "Capacity", "Category", "Visibility", "Attachments"
    ];
    const cols = [], vals = [];
    fields.forEach(f => {
      if (req.body[f] !== undefined) { cols.push(`${f} = ?`); vals.push(req.body[f]); }
    });
    if (!cols.length) return res.status(400).json({ error: "No data" });
    vals.push(id);
    db.run(
      `UPDATE ${SESS_TABLE} SET ${cols.join(", ")} WHERE SessionID = ?`,
      vals,
      function (err) {
        err
          ? res.status(500).json({ error: err.message })
          : res.json({ updated: this.changes });
      }
    );
  });

  // delete
  app.delete(["/api/session/:id", "/api/sessions/:id"], (req, res) => {
    db.run(
      `DELETE FROM ${SESS_TABLE} WHERE SessionID = ?`,
      [+req.params.id],
      function (err) {
        err
          ? res.status(500).json({ error: err.message })
          : res.json({ deleted: this.changes });
      }
    );
  });

  // sign-up
  app.post(["/api/session/:id/signup", "/api/sessions/:id/signup"], (req, res) => {
    if (!req.session?.studentID)
      return res.status(401).json({ error: "Login required" });

    const sessionID = +req.params.id;
    const studentID = req.session.studentID;

    db.get(
      `SELECT Capacity, NumSignedUp, SignedUpIDs
         FROM ${SESS_TABLE}
        WHERE SessionID = ?`,
      [sessionID],
      (err, row) => {
        if (err) return res.status(500).json({ error: err.message });
        if (!row) return res.status(404).json({ error: "Session not found" });

        const ids = JSON.parse(row.SignedUpIDs || "[]");
        if (ids.includes(studentID))
          return res.status(400).json({ error: "Already signed up" });
        if (row.Capacity && row.NumSignedUp >= row.Capacity)
          return res.status(400).json({ error: "Session full" });

        ids.push(studentID);
        db.run(
          `UPDATE ${SESS_TABLE}
              SET NumSignedUp = ?, SignedUpIDs = ?
            WHERE SessionID = ?`,
          [ids.length, JSON.stringify(ids), sessionID],
          function (uErr) {
            if (uErr) return res.status(500).json({ error: uErr.message });
            res.json({ ok: true, numSignedUp: ids.length });
          }
        );
      }
    );
  });

  /* -----------------------------------------------------------
     USER PROFILE – update only (GET handled earlier)
  ----------------------------------------------------------- */
  app.put("/api/user/me", (req, res) => {
    if (!req.session?.studentID) return res.status(401).json({ error: "Login required" });
    db.run(
      `UPDATE Users SET FirstName = ?, PhoneNumber = ? WHERE StudentID = ?`,
      [req.body.FirstName, req.body.PhoneNumber, req.session.studentID],
      function (err) {
        err
          ? res.status(500).json({ error: err.message })
          : res.json({ updated: this.changes });
      }
    );
  });

  /* ───────── PAGE routes ───────── */
  // public
  app.get("/landingPage.html", sendPage("landingPage.html"));
  app.get("/signup.html",      sendPage("signup.html"));
  app.get("/login.html",       sendPage("login.html"));
  // logged-in
  app.get("/about.html",       sendIfLoggedIn("about.html"));
  app.get("/data.html",        sendIfLoggedIn("data.html"));
  app.get("/session.html",     sendIfLoggedIn("session.html"));
  app.get("/inquiry.html",     sendIfLoggedIn("inquiry.html"));
  app.get("/join.html",        sendIfLoggedIn("join.html"));
  app.get("/profile.html",     sendIfLoggedIn("profile.html"));
  // admin
  app.get("/admin.html",       sendIfAdmin("admin.html"));
  // root
  app.get("/", (_req, res) => res.redirect("/landingPage.html"));
};
