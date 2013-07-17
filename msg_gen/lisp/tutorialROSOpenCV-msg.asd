
(cl:in-package :asdf)

(defsystem "tutorialROSOpenCV-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Stringts" :depends-on ("_package_Stringts"))
    (:file "_package_Stringts" :depends-on ("_package"))
  ))